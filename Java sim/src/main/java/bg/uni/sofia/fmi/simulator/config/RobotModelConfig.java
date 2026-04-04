package bg.uni.sofia.fmi.simulator.config;

public class RobotModelConfig {

    private double batteryCapacity;
    private double batteryConsumption;

    private double lidarRange;

    private double speed;

    // Getters & Setters

    public double getBatteryCapacity() {
        return batteryCapacity;
    }

    public void setBatteryCapacity(double batteryCapacity) {
        this.batteryCapacity = batteryCapacity;
    }

    public double getBatteryConsumption() {
        return batteryConsumption;
    }

    public void setBatteryConsumption(double batteryConsumption) {
        this.batteryConsumption = batteryConsumption;
    }

    public double getLidarRange() {
        return lidarRange;
    }

    public void setLidarRange(double lidarRange) {
        this.lidarRange = lidarRange;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}