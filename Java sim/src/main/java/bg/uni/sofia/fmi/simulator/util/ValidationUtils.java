package bg.uni.sofia.fmi.simulator.util;

import java.util.Objects;

/**
 * Помощен клас за валидиране на параметри и състояния в симулацията.
 */
public class ValidationUtils {

    // Проверка за вероятност в интервала [0.0, 1.0]
    public static double checkProbability(double value, String paramName) {
        if (Double.isNaN(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    (paramName != null ? paramName : "Probability") + " must be between 0.0 and 1.0. Given: " + value
            );
        }
        return value;
    }

    // Проверка за строго положително число (> 0)
    public static double checkPositive(double value, String paramName) {
        if (Double.isNaN(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    (paramName != null ? paramName : "Value") + " must be strictly positive (> 0). Given: " + value
            );
        }
        return value;
    }

    // Проверка за цяло строго положително число (> 0)
    public static int checkPositive(int value, String paramName) {
        if (value <= 0) {
            throw new IllegalArgumentException(
                    (paramName != null ? paramName : "Value") + " must be strictly positive (> 0). Given: " + value
            );
        }
        return value;
    }

    // Проверка за неотрицателно число (>= 0)
    public static double checkNonNegative(double value, String paramName) {
        if (Double.isNaN(value) || value < 0.0) {
            throw new IllegalArgumentException(
                    (paramName != null ? paramName : "Value") + " must be non-negative (>= 0). Given: " + value
            );
        }
        return value;
    }

    // Проверка за цяло неотрицателно число (>= 0)
    public static int checkNonNegative(int value, String paramName) {
        if (value < 0) {
            throw new IllegalArgumentException(
                    (paramName != null ? paramName : "Value") + " must be non-negative (>= 0). Given: " + value
            );
        }
        return value;
    }

    // Проверка за число в определен интервал [min, max]
    public static double checkRange(double value, double min, double max, String paramName) {
        if (min > max) {
            throw new IllegalArgumentException("min (" + min + ") cannot be greater than max (" + max + ")");
        }
        if (Double.isNaN(value) || value < min || value > max) {
            throw new IllegalArgumentException(
                    (paramName != null ? paramName : "Value") + " must be between " + min + " and " + max + ". Given: " + value
            );
        }
        return value;
    }

    // Проверка за цяло число в определен интервал [min, max]
    public static int checkRange(int value, int min, int max, String paramName) {
        if (min > max) {
            throw new IllegalArgumentException("min (" + min + ") cannot be greater than max (" + max + ")");
        }
        if (value < min || value > max) {
            throw new IllegalArgumentException(
                    (paramName != null ? paramName : "Value") + " must be between " + min + " and " + max + ". Given: " + value
            );
        }
        return value;
    }

    // Проверка за non-null референция
    public static <T> T checkNotNull(T value, String paramName) {
        if (value == null) {
            throw new IllegalArgumentException((paramName != null ? paramName : "Object") + " cannot be null");
        }
        return value;
    }
}
