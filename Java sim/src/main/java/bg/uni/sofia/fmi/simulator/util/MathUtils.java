package bg.uni.sofia.fmi.simulator.util;

import java.util.List;

//For statistics (used later for confidence intervals, etc.)
public class MathUtils {

    public static double mean(List<Double> values) {
        if (values.isEmpty()) return 0.0;

        double sum = 0;
        for (double v : values) sum += v;

        return sum / values.size();
    }

    public static double std(List<Double> values, double mean) {
        if (values.isEmpty()) return 0.0;

        double sum = 0;
        for (double v : values) {
            sum += Math.pow(v - mean, 2);
        }

        return Math.sqrt(sum / values.size());
    }
}
