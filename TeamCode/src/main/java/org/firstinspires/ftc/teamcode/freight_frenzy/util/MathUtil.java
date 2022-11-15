package org.firstinspires.ftc.teamcode.freight_frenzy.util;

public class MathUtil {

    public static double clamp(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        if (value < min) {
            return min;
        }
        return Math.min(value, max);
    }

    public static boolean inRange(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value > min && value < max;
    }

}
