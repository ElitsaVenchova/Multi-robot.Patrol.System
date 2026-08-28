package bg.uni.sofia.fmi.simulator.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// За изчисляване на статистически метрики като средно и стандартно отклонение
public class MathUtils {

    // Ограничаване на стойност в интервала [min, max]
    public static double clamp(double val, double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min cannot be greater than max");
        }
        return Math.max(min, Math.min(max, val));
    }

    // Метод за изчисляване на средната стойност от списък с числа
    public static double mean(List<Double> values) {
        if (values == null || values.isEmpty()) return 0.0;

        double sum = 0;
        for (double v : values) sum += v;

        return sum / values.size();
    }

    // Метод за изчисляване на стандартното отклонение (генерална съвкупност / population standard deviation)
    public static double std(List<Double> values, double mean) {
        if (values == null || values.isEmpty()) return 0.0;

        double sum = 0;
        for (double v : values) {
            sum += Math.pow(v - mean, 2);
        }

        return Math.sqrt(sum / values.size());
    }

    // Метод за изчисляване на извадковото стандартно отклонение (sample standard deviation, N - 1)
    public static double sampleStd(List<Double> values, double mean) {
        if (values == null || values.size() <= 1) return 0.0;

        double sum = 0;
        for (double v : values) {
            sum += Math.pow(v - mean, 2);
        }

        return Math.sqrt(sum / (values.size() - 1));
    }

    // Изчисляване на стандартна грешка на средната стойност (SEM = std / sqrt(N))
    public static double standardError(List<Double> values, double std) {
        if (values == null || values.isEmpty()) return 0.0;
        return std / Math.sqrt(values.size());
    }

    // Изчисляване на медиана от списък с числа
    public static double median(List<Double> values) {
        if (values == null || values.isEmpty()) return 0.0;

        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int n = sorted.size();
        if (n % 2 == 1) {
            return sorted.get(n / 2);
        } else {
            return (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2.0;
        }
    }

    // Намиране на минимална стойност от списък
    public static double min(List<Double> values) {
        if (values == null || values.isEmpty()) return 0.0;

        double min = values.get(0);
        for (int i = 1; i < values.size(); i++) {
            if (values.get(i) < min) {
                min = values.get(i);
            }
        }
        return min;
    }

    // Намиране на максимална стойност от списък
    public static double max(List<Double> values) {
        if (values == null || values.isEmpty()) return 0.0;

        double max = values.get(0);
        for (int i = 1; i < values.size(); i++) {
            if (values.get(i) > max) {
                max = values.get(i);
            }
        }
        return max;
    }
}
