plugins {
    alias(libs.plugins.android.library)
    `maven-publish`
}

group = "com.github.MohammadNouri5700"
version = "1.0.7"

kotlin {
    jvmToolchain(17)
}

android {
    namespace = "com.mohammadnouri5700.rtkrobotkalman"
    compileSdk = 35

    defaultConfig {
        minSdk = 26
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("consumer-rules.pro")
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }
    buildFeatures {
        viewBinding = true
    }
    publishing {
        singleVariant("release") {
            withSourcesJar()
            withJavadocJar()
        }
    }
}

dependencies {
    implementation(libs.androidx.appcompat)
    implementation(libs.androidx.fragment)
    implementation(libs.androidx.core.ktx)
    implementation(libs.kotlinx.coroutines.android)
    implementation(libs.material)
    
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.kotlinx.coroutines.test)
}

afterEvaluate {
    publishing {
        publications {
            create<MavenPublication>("release") {
                from(components["release"])
                groupId = project.group.toString()
                artifactId = "android-rtk-robot-kalman"
                version = project.version.toString()

                pom {
                    name.set("Android RTK Robot Kalman")
                    description.set("A high-performance sensor fusion library for Android using RTKLIB and Extended Kalman Filter.")
                    url.set("https://github.com/MohammadNouri5700/android-rtk-robot-kalman")
                    licenses {
                        license {
                            name.set("The Apache License, Version 2.0")
                            url.set("http://www.apache.org/licenses/LICENSE-2.0.txt")
                        }
                    }
                    developers {
                        developer {
                            id.set("MohammadNouri5700")
                            name.set("Mohammad Nouri")
                        }
                    }
                    scm {
                        connection.set("scm:git:github.com/MohammadNouri5700/android-rtk-robot-kalman.git")
                        developerConnection.set("scm:git:ssh://github.com/MohammadNouri5700/android-rtk-robot-kalman.git")
                        url.set("https://github.com/MohammadNouri5700/android-rtk-robot-kalman/tree/main")
                    }
                }
            }
        }
    }
}
