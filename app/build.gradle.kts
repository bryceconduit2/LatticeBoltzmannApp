plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.bc.fluidsandbox"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.bc.fluidsandbox"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"

        // 1. Configure the C++ compiler flags
        externalNativeBuild {
            cmake {
                cppFlags += listOf("-O3", "-ffast-math")
                abiFilters += setOf("arm64-v8a", "x86_64")
            }
        }
    }

    // 2. Point Gradle to your CMake script
    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }
    buildTypes {
        getByName("release") {
            isMinifyEnabled = false
            isShrinkResources = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")

            // REMOVED debug signing override for production security

            // ADD THIS BLOCK:
            installation {
                enableBaselineProfile = false
            }
        }
    }
    sourceSets {
        getByName("main") {
            java.srcDirs("src/main/java")
            jniLibs.srcDirs("src/main/jniLibs")
        }
        getByName("androidTest") {
            java.srcDirs("src/androidTest/java")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }
}

dependencies {

    implementation("androidx.core:core-ktx:1.9.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.10.0")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
    androidTestImplementation("androidx.test:rules:1.6.1")
}
