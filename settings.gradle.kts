pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()

        // Aliyun Mirrors (Backup)
        maven { url = uri("https://maven.aliyun.com/repository/google") }
        maven { url = uri("https://maven.aliyun.com/repository/public") }
        maven { url = uri("https://maven.aliyun.com/repository/gradle-plugin") }

        // Other custom endpoints
        maven { url = uri("https://maven.neshan.org/artifactory/public-maven") }
        maven { url = uri("https://dl.cloudsmith.io/public/carto/mobile-sdk/maven/") }
        maven { url = uri("https://jitpack.io") }
    }
}
plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version "1.0.0"
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        // Aliyun Mirrors (Backup) - Moved to top for reliability
        maven { url = uri("https://maven.aliyun.com/repository/google") }
        maven { url = uri("https://maven.aliyun.com/repository/public") }

        google()
        mavenCentral()

        maven { url = uri("https://maven.neshan.org/artifactory/public-maven") }
        maven { url = uri("https://dl.cloudsmith.io/public/carto/mobile-sdk/maven/") }
        maven { url = uri("https://jitpack.io") }
    }
}

rootProject.name = "android-rtk-robot-kalman"
include(":android-rtk-robot-kalman")
