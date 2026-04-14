package bg.uni.sofia.fmi.simulator.util;

import java.util.List;

// За изчисляване на статистически метрики като средно и стандартно отклонение
public class MathUtils {
    // Метод за изчисляване на средната стойност от списък с числа
    public static double mean(List<Double> values) {
        if (values.isEmpty()) return 0.0;

        double sum = 0;
        for (double v : values) sum += v;

        return sum / values.size();
    }
    // Метод за изчисляване на стандартното отклонение от списък с числа, като се използва средната стойност
    public static double std(List<Double> values, double mean) {
        if (values.isEmpty()) return 0.0;

        double sum = 0;
        for (double v : values) {
            sum += Math.pow(v - mean, 2);
        }

        return Math.sqrt(sum / values.size());
    }
}
