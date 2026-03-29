package bg.uni.sofia.fmi.simulator.results;

public class AggregatedMetrics {

    private double meanSuccessRate;
    private double stdSuccessRate;

    private double meanDetectionTime;
    private double stdDetectionTime;

    private int runs;

    // Getters & Setters

    public double getMeanSuccessRate() {
        return meanSuccessRate;
    }

    public void setMeanSuccessRate(double meanSuccessRate) {
        this.meanSuccessRate = meanSuccessRate;
    }

    public double getStdSuccessRate() {
        return stdSuccessRate;
    }

    public void setStdSuccessRate(double stdSuccessRate) {
        this.stdSuccessRate = stdSuccessRate;
    }

    public double getMeanDetectionTime() {
        return meanDetectionTime;
    }

    public void setMeanDetectionTime(double meanDetectionTime) {
        this.meanDetectionTime = meanDetectionTime;
    }

    public double getStdDetectionTime() {
        return stdDetectionTime;
    }

    public void setStdDetectionTime(double stdDetectionTime) {
        this.stdDetectionTime = stdDetectionTime;
    }

    public int getRuns() {
        return runs;
    }

    public void setRuns(int runs) {
        this.runs = runs;
    }
}