package bg.uni.sofia.fmi.simulator.results;

import java.util.List;

// Този клас ще съхранява всички метрики, изчислени след изпълнението на симулацията.
// Това е контейнер за данни (DTO) за резултатите.
public class SimulationMetrics {
    private double captureCoefficient = Double.NaN;
    private double activityCoefficient = Double.NaN;
    private long activeAttacks;
    private List<RobotMetrics> robots = List.of();
    private double robotAcquisitionCost, stationAcquisitionCost;
    private double energyCost = Double.NaN, batteryReplacementCost = Double.NaN;

    private long totalAttacks;
    private int interceptedAttacks;
    private int missedAttacks;
    private double successRate;
    private double averageDetectionTime;
    private double averageResponseTime;
    private double detectionRate;

    // Getters & Setters
    public long getTotalAttacks() { return totalAttacks; }
    public void setTotalAttacks(long totalAttacks) { this.totalAttacks = totalAttacks;}

    public int getInterceptedAttacks() { return interceptedAttacks; }
    public void setInterceptedAttacks(int interceptedAttacks) { this.interceptedAttacks = interceptedAttacks;}

    public int getMissedAttacks() { return missedAttacks; }
    public void setMissedAttacks(int missedAttacks) { this.missedAttacks = missedAttacks;}

    public double getSuccessRate() { return successRate; }
    public void setSuccessRate(double successRate) { this.successRate = successRate;}

    public double getAverageDetectionTime() { return averageDetectionTime; }
    public void setAverageDetectionTime(double averageDetectionTime) { this.averageDetectionTime = averageDetectionTime;}

    public double getAverageResponseTime() { return averageResponseTime; }
    public void setAverageResponseTime(double averageResponseTime) { this.averageResponseTime = averageResponseTime;}

    public double getDetectionRate() { return detectionRate; }
    public void setDetectionRate(double detectionRate) { this.detectionRate = detectionRate;}

    public double getCaptureCoefficient() { return captureCoefficient; }
    public void setCaptureCoefficient(double v) { captureCoefficient = v; }

    public double getActivityCoefficient() { return activityCoefficient; }
    public void setActivityCoefficient(double v) { activityCoefficient = v; }

    public long getActiveAttacks() { return activeAttacks; }
    public void setActiveAttacks(long v) { activeAttacks = v; }

    public List<RobotMetrics> getRobots() { return robots; }
    public void setRobots(List<RobotMetrics> v) { robots = List.copyOf(v); }

    public double getRobotAcquisitionCost() { return robotAcquisitionCost; }
    public void setRobotAcquisitionCost(double v) { robotAcquisitionCost = v; }

    public double getStationAcquisitionCost() { return stationAcquisitionCost; }
    public void setStationAcquisitionCost(double v) { stationAcquisitionCost = v; }

    public double getEnergyCost() { return energyCost; }
    public void setEnergyCost(double v) { energyCost = v; }

    public double getBatteryReplacementCost() { return batteryReplacementCost; }
    public void setBatteryReplacementCost(double v) { batteryReplacementCost = v; }

    public double getTco() { return robotAcquisitionCost + stationAcquisitionCost + energyCost + batteryReplacementCost; }
}