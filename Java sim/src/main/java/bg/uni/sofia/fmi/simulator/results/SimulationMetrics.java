package bg.uni.sofia.fmi.simulator.results;

// This class will hold all the metrics calculated after the simulation run
// This is a data container (DTO) for results.
public class SimulationMetrics {

    private int totalAttacks;
    private int interceptedAttacks;
    private int missedAttacks;
    private double successRate;
    private double averageDetectionTime;
    private double averageResponseTime;
    private double detectionRate;

    // Getters & Setters

    public int getTotalAttacks() {
        return totalAttacks;
    }

    public void setTotalAttacks(int totalAttacks) {
        this.totalAttacks = totalAttacks;
    }

    public int getInterceptedAttacks() {
        return interceptedAttacks;
    }

    public void setInterceptedAttacks(int interceptedAttacks) {
        this.interceptedAttacks = interceptedAttacks;
    }

    public int getMissedAttacks() {
        return missedAttacks;
    }

    public void setMissedAttacks(int missedAttacks) {
        this.missedAttacks = missedAttacks;
    }

    public double getSuccessRate() {
        return successRate;
    }

    public void setSuccessRate(double successRate) {
        this.successRate = successRate;
    }

    public double getAverageDetectionTime() {
        return averageDetectionTime;
    }

    public void setAverageDetectionTime(double averageDetectionTime) {
        this.averageDetectionTime = averageDetectionTime;
    }

    public double getAverageResponseTime() {
        return averageResponseTime;
    }

    public void setAverageResponseTime(double averageResponseTime) {
        this.averageResponseTime = averageResponseTime;
    }

    public double getDetectionRate() {
        return detectionRate;
    }

    public void setDetectionRate(double detectionRate) {
        this.detectionRate = detectionRate;
    }
}