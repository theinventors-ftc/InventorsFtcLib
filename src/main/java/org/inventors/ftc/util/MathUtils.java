package org.inventors.ftc.util;

public class MathUtils {
    private MathUtils() {
        throw new AssertionError("utility class");
    }
    public static double sum(double... nums) {
        double total = 0;
        for(double num : nums) {
            total += num;
        }
        return total;
    }

    public static double average(double... nums) {
        return nums.length == 0 ? 0.0 : sum(nums) / nums.length;
    }

    public static double round(double num, double precision)
    {
        return Math.round(num / precision) * precision;
    }

    // inRange function for int int int, double double double, int double double, double int int
    public static boolean inRange(int value, int low, int high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }

    public static boolean inRange(double value, double low, double high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }

    public static boolean inRange(double value, int low, int high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }

    public static boolean inRange(int value, double low, double high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }

    public static boolean inRange(int value, int low, int high)
    {
        return inRange(value, low, high, true);
    }

    public static boolean inRange(double value, double low, double high)
    {
        return inRange(value, low, high, true);
    }

    public static boolean inRange(double value, int low, int high)
    {
        return inRange(value, low, high, true);
    }

    public static boolean inRange(int value, double low, double high)
    {
        return inRange(value, low, high, true);
    }
}
