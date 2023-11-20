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

    public static int clamp(int value, int min, int max) {
        if (min > max) {
            int tempMin = min;
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
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value <= min || value >= max;
    }

    public static boolean approximately(double value, double targetValue, double approximation) {
        return value > (targetValue - approximation) && value < (targetValue + approximation);
    }
}
