package org.firstinspires.ftc.teamcode.Util;

public class MathUtil {

    public static double clamp(double min, double max, double t) {
        if (t < min) return min;
        if (t > max) return max;
        return t;
    }

    public static double lerp(double min, double max, double t) {
        return clamp(min, max, min * (1.0 - t) + max * t);
    }
    public static double invLerp(double min, double max, double t) {
        return clamp(0.0, 1.0, (t - min) / (max - min));
    }
    public static double remap(double iMin, double iMax, double oMin, double oMax, double t) {
        return lerp(oMin, oMax, invLerp(iMin, iMax, t));
    }

}
