package bg.uni.sofia.fmi.simulator.results;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

// This class will calculate all the metrics after the simulation run
public class MetricsCalculator {

    public SimulationMetrics calculate(World world) {

        SimulationMetrics metrics = new SimulationMetrics();

        int total = world.getAttacks().size();
        int intercepted = 0;
        int missed = 0;

        double totalDetectionTime = 0.0;
        int detectedCount = 0;

        for (Attack attack : world.getAttacks()) {

            if (attack.getStatus() == AttackStatus.INTERCEPTED) {
                intercepted++;

                int detectionTime = attack.getDetectionTime();
                int creationTime = attack.getCreationTime();

                if (detectionTime >= 0) {
                    totalDetectionTime += (detectionTime - creationTime);
                    detectedCount++;
                }

            } else if (attack.getStatus() == AttackStatus.MISSED) {
                missed++;
            }
        }

        metrics.setTotalAttacks(total);
        metrics.setInterceptedAttacks(intercepted);
        metrics.setMissedAttacks(missed);

        // Success rate
        if (total > 0) {
            metrics.setSuccessRate((double) intercepted / total);
            metrics.setDetectionRate((double) intercepted / total); // [TODO] same for now
        } else {
            metrics.setSuccessRate(0.0);
            metrics.setDetectionRate(0.0);
        }

        // Average detection time
        if (detectedCount > 0) {
            double avgDetection = totalDetectionTime / detectedCount;
            metrics.setAverageDetectionTime(avgDetection);
            metrics.setAverageResponseTime(avgDetection); // [TODO] same for now  
        } else {
            metrics.setAverageDetectionTime(0.0);
            metrics.setAverageResponseTime(0.0);
        }

        return metrics;
    }
}