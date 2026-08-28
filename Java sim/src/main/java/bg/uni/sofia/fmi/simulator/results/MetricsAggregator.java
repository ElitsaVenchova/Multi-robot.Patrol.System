package bg.uni.sofia.fmi.simulator.results;

import bg.uni.sofia.fmi.simulator.util.MathUtils;

import java.util.ArrayList;
import java.util.List;

public class MetricsAggregator {

    public AggregatedMetrics aggregate(List<SimulationMetrics> metricsList) {

        AggregatedMetrics result = new AggregatedMetrics();

        int n = metricsList.size();
        result.setRuns(n);

        if (n == 0) return result;

        List<Double> successRates = new ArrayList<>(n);
        List<Double> detectionTimes = new ArrayList<>(n);

        for (SimulationMetrics m : metricsList) {
            successRates.add(m.getSuccessRate());
            detectionTimes.add(m.getAverageDetectionTime());
        }

        // ---- Means ----
        double meanSuccess = MathUtils.mean(successRates);
        double meanDetection = MathUtils.mean(detectionTimes);

        result.setMeanSuccessRate(meanSuccess);
        result.setMeanDetectionTime(meanDetection);

        // ---- Standard deviation ----
        double stdSuccess = MathUtils.std(successRates, meanSuccess);
        double stdDetection = MathUtils.std(detectionTimes, meanDetection);

        result.setStdSuccessRate(stdSuccess);
        result.setStdDetectionTime(stdDetection);

        return result;
    }
}
