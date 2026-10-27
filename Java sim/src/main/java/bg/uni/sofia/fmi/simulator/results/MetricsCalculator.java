package bg.uni.sofia.fmi.simulator.results;

import bg.uni.sofia.fmi.simulator.config.SimulationConfig;
import bg.uni.sofia.fmi.simulator.domain.*;
import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

import java.util.ArrayList;
import java.util.List;

// Този клас ще изчисли всички метрики след изпълнението на симулацията.
public class MetricsCalculator {
    /** Добавя експериментални метрики, като същевременно запазва досегашния
     * показател successRate = intercepted/generated. */
    public SimulationMetrics calculate(SimulationConfig config) {
        World.finish();
        SimulationMetrics metrics = calculate();
        long resolved = (long) metrics.getInterceptedAttacks() + metrics.getMissedAttacks();
        metrics.setCaptureCoefficient(resolved == 0 ? Double.NaN :
                (double) metrics.getInterceptedAttacks() / resolved);
        metrics.setActiveAttacks(metrics.getTotalAttacks() - resolved);
        if (metrics.getInterceptedAttacks() == 0) metrics.setAverageDetectionTime(Double.NaN);
        List<RobotMetrics> robots = new ArrayList<>();
        for (Bot bot : World.getBots()) {
            Battery battery = bot.getBattery();
            robots.add(new RobotMetrics(bot.getId(), bot.getName(), bot.getType().name(), bot.getStateTicks(),
                    battery.getChargedWh(), battery.getDischargedWh(), battery.getEquivalentDischargeCycles(),
                    battery.getWearCostEur()));
        }
        metrics.setRobots(robots);
        long patrol = robots.stream().mapToLong(RobotMetrics::patrolMinutes).sum();
        long total = robots.stream().mapToLong(RobotMetrics::totalMinutes).sum();
        long nonPatrol = total - patrol;
        metrics.setActivityCoefficient(nonPatrol == 0 ? Double.NaN : (double) patrol / nonPatrol);
        metrics.setRobotAcquisitionCost(World.getBots().stream().mapToDouble(Bot::getPrice).sum());
        metrics.setStationAcquisitionCost(World.getChargingStations().stream()
                .mapToDouble(ChargingStation::getPrice).sum());
        Double tariff = config.getEnergyPriceEurPerKwh();
        metrics.setEnergyCost(tariff == null ? Double.NaN :
                robots.stream().mapToDouble(RobotMetrics::chargedWh).sum() / 1000.0 * tariff);
        metrics.setBatteryReplacementCost(robots.stream().mapToDouble(RobotMetrics::batteryWearEur).sum());
        return metrics;
    }

    public SimulationMetrics calculate() {

        SimulationMetrics metrics = new SimulationMetrics();

        long total = World.getPerimeter().streamAttacks().count();
        int intercepted = 0;
        int missed = 0;

        double totalDetectionTime = 0.0;
        int detectedCount = 0;

        for (Attack attack : World.getPerimeter().streamAttacks().toList()) {

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
