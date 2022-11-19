package org.firstinspires.ftc.teamcode.freight_frenzy.util;

public class MathUtil {

    public static double clamp(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return Math.max(Math.min(value, max), min);
    }

    public static boolean inRange(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value > min && value < max;
    }

    public static boolean outOfRange(double value, double min, double max) {
        if(min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value <= min || value >= max;
    }

    public boolean approximately(double value, double valuedAt, double approximation) {
        return value > (valuedAt - approximation) && value < (valuedAt + approximation);
    }
}
