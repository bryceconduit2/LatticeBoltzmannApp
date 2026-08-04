# Add project specific ProGuard rules here.
# You can control the set of applied configuration files using the
# proguardFiles setting in build.gradle.
#
# For more details, see
#   http://developer.android.com/guide/developing/tools/proguard.html

# If your project uses WebView with JS, uncomment the following
# and specify the fully qualified class name to the JavaScript interface
# class:
#-keepclassmembers class fqcn.of.javascript.interface.for.webview {
#   public *;
#}

# Keep the Native engine class and its members for JNI
-keep class com.bc.fluidsandbox.NativeLBMEngine {
    *;
}

# Keep Custom Views to prevent layout inflation errors
-keep class com.bc.fluidsandbox.WindTunnelView { *; }
-keep class com.bc.fluidsandbox.ForceGraphView { *; }

# Keep the Native methods
-keepclasseswithmembernames class * {
    native <methods>;
}

# Uncomment this to preserve the line number information for
# debugging stack traces.
-keepattributes SourceFile,LineNumberTable

# Strip Debug and Verbose logs for Production performance/privacy
-assumenosideeffects class android.util.Log {
    public static *** d(...);
    public static *** v(...);
}

# If you keep the line number information, uncomment this to
# hide the original source file name.
#-renamesourcefileattribute SourceFile
