import os
import time
import random
import urllib.request
import json
import math
from collections import deque

# Start and End coordinates
start_lat, start_lon = 35.757677, 51.414663
end_lat, end_lon = 35.743685, 51.411803

steps = 240  # 240 GPS steps (120 seconds of driving at 0.5s intervals)
dt = 0.5     # GPS time step in seconds

# ============================================================
# GPS NOISE / STRESS-TEST CONFIGURATION
# ============================================================
NOISE_STD_DEV_METERS = 8.0
DRIFT_CORRELATION = 0.95
QUALITY_VARIATION = True
QUALITY_CYCLE_STEPS = 60
QUALITY_MIN_MULT = 0.5
QUALITY_MAX_MULT = 3.5
OUTLIER_ENABLED = True
OUTLIER_PROBABILITY = 0.04
OUTLIER_MIN_METERS = 20.0
OUTLIER_MAX_METERS = 80.0
OUTLIER_DECAY_STEPS = 3
DROPOUT_ENABLED = True
DROPOUT_PROBABILITY = 0.01
DROPOUT_MIN_STEPS = 4
DROPOUT_MAX_STEPS = 10
DROPOUT_JUMP_ON_RECOVERY_METERS = 40.0
NOISE_THRESHOLD_METERS = 150.0

# --- GPS receiver processing latency (real receivers report a fix ~0.1-0.5s
#     after it was actually acquired - your filter should account for this) ---
GPS_LATENCY_STEPS = 1          # delay GPS output by N main-loop steps (0 = disable)

# --- Time-to-first-fix: real GPS chips take a few seconds to get a cold-start fix ---
TTFF_SECONDS = 3.0              # set to 0 to disable

# ============================================================
# IMU CONFIGURATION
# ============================================================
IMU_SUBSTEPS = 10               # IMU updates this many times per GPS step
                                 # e.g. dt=0.5s, IMU_SUBSTEPS=10 -> IMU @ 20Hz, GPS @ 2Hz
                                 # (raise this for a faster IMU, but os.system/adb overhead
                                 #  may not keep up in real time above ~50-100Hz)
IMU_DT = dt / IMU_SUBSTEPS

ENGINE_VIBRATION_G = 0.15       # baseline high-freq vibration noise (m/s^2, 1-sigma)
ROAD_BUMP_G_MAX = 3.0           # max vertical bump (m/s^2)
BUMP_PROBABILITY = 0.02         # chance of a bump per IMU substep (scaled to higher rate)
GYRO_JITTER_RAD = 0.03          # baseline gyro white noise (rad/s, 1-sigma)

# --- Sensor bias instability (slow random-walk bias - THE classic real-IMU behavior) ---
ACCEL_BIAS_WALK_STD = 0.002     # m/s^2 per sqrt(step) - how fast accel bias wanders
GYRO_BIAS_WALK_STD = 0.0005     # rad/s per sqrt(step) - how fast gyro bias wanders
ACCEL_BIAS_MAX = 0.3            # clamp so bias doesn't wander unboundedly
GYRO_BIAS_MAX = 0.05

# --- Colored (correlated) vibration noise via an AR(1)/Ornstein-Uhlenbeck process,
#     since real vibration isn't independent sample-to-sample ---
VIBRATION_CORRELATION = 0.7     # 0 = white noise, closer to 1 = smoother/slower oscillation

# --- Magnetometer simulation (for heading/yaw fusion testing) ---
MAGNETOMETER_ENABLED = True
MAG_FIELD_MAGNITUDE_UT = 45.0   # total field strength, microtesla (typical mid-latitude)
MAG_INCLINATION_DEG = 55.0      # dip angle at this latitude (approx for Tehran region)
MAG_DECLINATION_DEG = 4.5       # local declination (magnetic north vs true north)
MAG_NOISE_STD_UT = 1.0          # sensor noise
MAG_HARD_IRON_OFFSET_UT = (2.0, -1.5, 0.5)  # fixed calibration offset per axis (device/vehicle magnetism)

# ------------------------------------------------------------
# Internal state (do not edit)
# ------------------------------------------------------------
current_drift_lat_m = 0.0
current_drift_lon_m = 0.0
_outlier_remaining_steps = 0
_outlier_vec = (0.0, 0.0)
_dropout_remaining_steps = 0
_dropout_frozen_pos = None
_just_recovered = False
_gps_output_buffer = deque()
_accel_bias = [0.0, 0.0, 0.0]
_gyro_bias = [0.0, 0.0, 0.0]
_vib_state = [0.0, 0.0, 0.0]   # colored vibration state per accel axis
_first_run = True

print("Fetching forward and return street routes...")


def get_route(slat, slon, elat, elon):
    url = f"http://router.project-osrm.org/route/v1/driving/{slon},{slat};{elon},{elat}?overview=full&geometries=geojson"
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    response = urllib.request.urlopen(req)
    data = json.loads(response.read().decode())
    return [[c[1], c[0]] for c in data['routes'][0]['geometry']['coordinates']]


route_forward = get_route(start_lat, start_lon, end_lat, end_lon)
route_backward = get_route(end_lat, end_lon, start_lat, start_lon)


def haversine(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    a = math.sin(math.radians(lat2 - lat1) / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Forward bearing from point 1 to point 2, in radians."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    return math.atan2(x, y)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def signal_quality_multiplier(step_index):
    if not QUALITY_VARIATION:
        return 1.0
    phase = (step_index % QUALITY_CYCLE_STEPS) / QUALITY_CYCLE_STEPS
    osc = (1 - math.cos(2 * math.pi * phase)) / 2.0
    return QUALITY_MIN_MULT + osc * (QUALITY_MAX_MULT - QUALITY_MIN_MULT)


def maybe_trigger_outlier():
    global _outlier_remaining_steps, _outlier_vec
    if _outlier_remaining_steps > 0:
        decay_fraction = _outlier_remaining_steps / OUTLIER_DECAY_STEPS
        _outlier_remaining_steps -= 1
        return (_outlier_vec[0] * decay_fraction, _outlier_vec[1] * decay_fraction)

    if OUTLIER_ENABLED and random.random() < OUTLIER_PROBABILITY:
        mag = random.uniform(OUTLIER_MIN_METERS, OUTLIER_MAX_METERS)
        angle = random.uniform(0, 2 * math.pi)
        _outlier_vec = (mag * math.cos(angle), mag * math.sin(angle))
        _outlier_remaining_steps = OUTLIER_DECAY_STEPS
        return _outlier_vec
    return (0.0, 0.0)


def maybe_trigger_dropout(clean_lat, clean_lon):
    global _dropout_remaining_steps, _dropout_frozen_pos, _just_recovered
    if _dropout_remaining_steps > 0:
        _dropout_remaining_steps -= 1
        if _dropout_remaining_steps == 0:
            _just_recovered = True
        return True, _dropout_frozen_pos[0], _dropout_frozen_pos[1]

    if DROPOUT_ENABLED and random.random() < DROPOUT_PROBABILITY:
        _dropout_remaining_steps = random.randint(DROPOUT_MIN_STEPS, DROPOUT_MAX_STEPS)
        _dropout_frozen_pos = (clean_lat, clean_lon)
        return True, clean_lat, clean_lon
    return False, clean_lat, clean_lon


def update_bias(bias, walk_std, max_abs):
    """Slow random-walk bias, clamped so it doesn't diverge unboundedly.
    This is what makes IMU integration drift over time in real hardware."""
    for i in range(3):
        bias[i] += random.gauss(0, walk_std)
        bias[i] = max(-max_abs, min(max_abs, bias[i]))
    return bias


def colored_noise_step(state, std_dev):
    """AR(1)/Ornstein-Uhlenbeck style colored noise: correlated sample-to-sample
    instead of independent white noise, closer to real vibration spectra."""
    for i in range(3):
        white = random.gauss(0, std_dev)
        state[i] = VIBRATION_CORRELATION * state[i] + (1 - VIBRATION_CORRELATION) * white
    return state


def send_magnetometer(heading_rad):
    """Simulates a magnetometer reading given current vehicle heading, including
    hard-iron calibration offset and sensor noise. Lets you test heading/yaw
    fusion (e.g. gyro + mag complementary/EKF fusion) rather than GPS alone."""
    if not MAGNETOMETER_ENABLED:
        return

    incl = math.radians(MAG_INCLINATION_DEG)
    decl = math.radians(MAG_DECLINATION_DEG)

    horizontal = MAG_FIELD_MAGNITUDE_UT * math.cos(incl)
    vertical = MAG_FIELD_MAGNITUDE_UT * math.sin(incl)

    mag_north = horizontal * math.cos(decl)
    mag_east = horizontal * math.sin(decl)

    # Project geographic field onto vehicle body frame (x = forward, y = right)
    body_x = mag_north * math.cos(heading_rad) + mag_east * math.sin(heading_rad)
    body_y = -mag_north * math.sin(heading_rad) + mag_east * math.cos(heading_rad)
    body_z = vertical

    body_x += MAG_HARD_IRON_OFFSET_UT[0] + random.gauss(0, MAG_NOISE_STD_UT)
    body_y += MAG_HARD_IRON_OFFSET_UT[1] + random.gauss(0, MAG_NOISE_STD_UT)
    body_z += MAG_HARD_IRON_OFFSET_UT[2] + random.gauss(0, MAG_NOISE_STD_UT)

    os.system(f"adb emu sensor set magnetic-field {body_x:.3f}:{body_y:.3f}:{body_z:.3f} > /dev/null 2>&1")


def drive_route(route_coords, direction_name):
    global current_drift_lat_m, current_drift_lon_m, _just_recovered, _first_run

    segments = [haversine(route_coords[i][0], route_coords[i][1], route_coords[i + 1][0], route_coords[i + 1][1]) for i in range(len(route_coords) - 1)]
    total_dist = sum(segments)
    step_dist = total_dist / steps

    print(f"\nSimulating {direction_name} drive ({total_dist:.0f}m) with GPS & IMU noise...")

    if _first_run and TTFF_SECONDS > 0:
        print(f"Waiting {TTFF_SECONDS:.1f}s for simulated GPS time-to-first-fix...")
        time.sleep(TTFF_SECONDS)
        _first_run = False

    current_segment = 0
    dist_along_segment = 0

    prev_clean_lat, prev_clean_lon = None, None
    prev_velocity, prev_heading = 0.0, 0.0
    accel_z_report = 9.81

    for i in range(steps + 1):
        if i == 0:
            clean_lat, clean_lon = route_coords[0]
        elif i == steps:
            clean_lat, clean_lon = route_coords[-1]
        else:
            target_dist = step_dist
            while target_dist > 0 and current_segment < len(segments):
                rem_segment = segments[current_segment] - dist_along_segment
                if target_dist <= rem_segment:
                    fraction = (dist_along_segment + target_dist) / segments[current_segment] if segments[current_segment] > 0 else 0
                    p1, p2 = route_coords[current_segment], route_coords[current_segment + 1]
                    clean_lat = p1[0] + (p2[0] - p1[0]) * fraction
                    clean_lon = p1[1] + (p2[1] - p1[1]) * fraction
                    dist_along_segment += target_dist
                    target_dist = 0
                else:
                    target_dist -= rem_segment
                    current_segment += 1
                    dist_along_segment = 0

        # ====================================================
        # 1. MACRO KINEMATICS (once per GPS-rate step)
        # ====================================================
        forward_accel = 0.0
        centripetal_accel = 0.0
        yaw_rate = 0.0
        heading = prev_heading

        if prev_clean_lat is not None:
            dist = haversine(prev_clean_lat, prev_clean_lon, clean_lat, clean_lon)
            velocity = dist / dt
            forward_accel = (velocity - prev_velocity) / dt

            if dist > 0.1:
                heading = calculate_bearing(prev_clean_lat, prev_clean_lon, clean_lat, clean_lon)

            yaw_rate = normalize_angle(heading - prev_heading) / dt
            centripetal_accel = velocity * yaw_rate

            prev_velocity = velocity
            prev_heading = heading

        # ====================================================
        # 2. IMU: stream at a higher rate than GPS, with bias
        #    random-walk + colored vibration + white noise
        # ====================================================
        global _accel_bias, _gyro_bias, _vib_state
        for _ in range(IMU_SUBSTEPS if prev_clean_lat is not None else 0):
            _accel_bias = update_bias(_accel_bias, ACCEL_BIAS_WALK_STD, ACCEL_BIAS_MAX)
            _gyro_bias = update_bias(_gyro_bias, GYRO_BIAS_WALK_STD, GYRO_BIAS_MAX)
            _vib_state = colored_noise_step(_vib_state, ENGINE_VIBRATION_G)

            bump_z = random.uniform(-ROAD_BUMP_G_MAX, ROAD_BUMP_G_MAX) if random.random() < BUMP_PROBABILITY else 0.0

            accel_x = centripetal_accel + _vib_state[0] + _accel_bias[0] + random.gauss(0, ENGINE_VIBRATION_G * 0.3)
            accel_y = forward_accel + _vib_state[1] + _accel_bias[1] + random.gauss(0, ENGINE_VIBRATION_G * 0.3)
            accel_z = 9.81 + bump_z + _vib_state[2] + _accel_bias[2] + random.gauss(0, ENGINE_VIBRATION_G * 0.3)
            accel_z_report = accel_z

            gyro_x = _gyro_bias[0] + random.gauss(0, GYRO_JITTER_RAD)
            gyro_y = _gyro_bias[1] + random.gauss(0, GYRO_JITTER_RAD)
            gyro_z = yaw_rate + _gyro_bias[2] + random.gauss(0, GYRO_JITTER_RAD)

            os.system(f"adb emu sensor set acceleration {accel_x:.3f}:{accel_y:.3f}:{accel_z:.3f} > /dev/null 2>&1")
            os.system(f"adb emu sensor set gyroscope {gyro_x:.3f}:{gyro_y:.3f}:{gyro_z:.3f} > /dev/null 2>&1")
            send_magnetometer(heading)

            time.sleep(IMU_DT)

        prev_clean_lat, prev_clean_lon = clean_lat, clean_lon

        # ====================================================
        # 3. GPS LOCATION STRESS TEST (unchanged model + latency buffer)
        # ====================================================
        dropped, base_lat, base_lon = maybe_trigger_dropout(clean_lat, clean_lon)
        quality_mult = signal_quality_multiplier(i)

        raw_noise_lat = random.gauss(0, NOISE_STD_DEV_METERS * quality_mult)
        raw_noise_lon = random.gauss(0, NOISE_STD_DEV_METERS * quality_mult)

        current_drift_lat_m = (DRIFT_CORRELATION * current_drift_lat_m) + ((1 - DRIFT_CORRELATION) * raw_noise_lat)
        current_drift_lon_m = (DRIFT_CORRELATION * current_drift_lon_m) + ((1 - DRIFT_CORRELATION) * raw_noise_lon)

        total_noise_lat_m = current_drift_lat_m + random.gauss(0, (NOISE_STD_DEV_METERS * quality_mult) / 2.0)
        total_noise_lon_m = current_drift_lon_m + random.gauss(0, (NOISE_STD_DEV_METERS * quality_mult) / 2.0)

        noise_distance = math.sqrt(total_noise_lat_m ** 2 + total_noise_lon_m ** 2)
        if noise_distance > NOISE_THRESHOLD_METERS:
            scale = NOISE_THRESHOLD_METERS / noise_distance
            total_noise_lat_m *= scale
            total_noise_lon_m *= scale

        outlier_lat_m, outlier_lon_m = maybe_trigger_outlier()
        total_noise_lat_m += outlier_lat_m
        total_noise_lon_m += outlier_lon_m

        if _just_recovered:
            angle = random.uniform(0, 2 * math.pi)
            total_noise_lat_m += DROPOUT_JUMP_ON_RECOVERY_METERS * math.cos(angle)
            total_noise_lon_m += DROPOUT_JUMP_ON_RECOVERY_METERS * math.sin(angle)
            _just_recovered = False

        lat_conversion = 1.0 / 111111.0
        lon_conversion = 1.0 / (111111.0 * math.cos(math.radians(base_lat)))

        noisy_lat = base_lat + (total_noise_lat_m * lat_conversion)
        noisy_lon = base_lon + (total_noise_lon_m * lon_conversion)

        # --- Apply receiver processing latency: delay the reported fix ---
        status = "DROPOUT" if dropped else ("OUTLIER" if (outlier_lat_m or outlier_lon_m) else "normal")
        _gps_output_buffer.append((noisy_lat, noisy_lon, status))

        if len(_gps_output_buffer) > GPS_LATENCY_STEPS:
            out_lat, out_lon, out_status = _gps_output_buffer.popleft()
            os.system(f"adb emu geo fix {out_lon:.6f} {out_lat:.6f} > /dev/null 2>&1")
            print(f"[{i}/{steps}] {out_status:8s} -> L:{out_lat:.6f},{out_lon:.6f}  az:{accel_z_report:.2f}m/s2", end='\r')

    print(f"\nArrived at {direction_name} destination!")


print("Starting continuous loop... Press Ctrl+C to stop.")
while True:
    drive_route(route_forward, "Forward")
    time.sleep(2)
    drive_route(route_backward, "Return")
    time.sleep(2)