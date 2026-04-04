package bg.uni.sofia.fmi.simulator.config;

public class RobotModelConfig {

    private String name;
    private String type; // "GROUND" or "DRONE"

    private double batteryCapacity;
    private double batteryConsumption;

    private double lidarRange;
    private double lidarBatteryConsumptionRate;

    private double maxSpeed;
    private double price;
    private double failureProbability;
    private double batteryConsumptionRate;

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

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double speed) {
        this.maxSpeed = speed;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public double getPrice() {
        return price;
    }

    public void setPrice(double price) {
        this.price = price;
    }

    public double getFailureProbability() {
        return failureProbability;
    }

    public void setFailureProbability(double failureProbability) {
        this.failureProbability = failureProbability;
    }

    public double getBatteryConsumptionRate() {
        return batteryConsumptionRate;
    }

    public void setBatteryConsumptionRate(double batteryConsumptionRate) {
        this.batteryConsumptionRate = batteryConsumptionRate;
    }

    public double getLidarBatteryConsumptionRate() {
        return lidarBatteryConsumptionRate;
    }

    public void setLidarBatteryConsumptionRate(double lidarBatteryConsumptionRate) {
        this.lidarBatteryConsumptionRate = lidarBatteryConsumptionRate;
    }
}