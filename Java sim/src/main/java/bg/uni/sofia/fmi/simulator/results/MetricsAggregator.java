package bg.uni.sofia.fmi.simulator.results;

import java.util.List;

public class MetricsAggregator {

    public AggregatedMetrics aggregate(List<SimulationMetrics> metricsList) {

        AggregatedMetrics result = new AggregatedMetrics();

        int n = metricsList.size();
        result.setRuns(n);

        if (n == 0) return result;

        // ---- Means ----
        double sumSuccess = 0.0;
        double sumDetection = 0.0;

        for (SimulationMetrics m : metricsList) {
            sumSuccess += m.getSuccessRate();
            sumDetection += m.getAverageDetectionTime();
        }

        double meanSuccess = sumSuccess / n;
        double meanDetection = sumDetection / n;

        result.setMeanSuccessRate(meanSuccess);
        result.setMeanDetectionTime(meanDetection);

        // ---- Standard deviation ----
        double varSuccess = 0.0;
        double varDetection = 0.0;

        for (SimulationMetrics m : metricsList) {
            varSuccess += Math.pow(m.getSuccessRate() - meanSuccess, 2);
            varDetection += Math.pow(m.getAverageDetectionTime() - meanDetection, 2);
        }

        double stdSuccess = Math.sqrt(varSuccess / n);
        double stdDetection = Math.sqrt(varDetection / n);

        result.setStdSuccessRate(stdSuccess);
        result.setStdDetectionTime(stdDetection);

        return result;
    }
}
