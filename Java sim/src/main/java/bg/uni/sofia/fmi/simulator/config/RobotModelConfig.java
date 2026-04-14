package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за моделите на роботите
public class RobotModelConfig {

    private String name; //името на модела
    private String type; // "GROUND" or "DRONE"
    private double batteryCapacity; // в единици, например mAh
    private double batteryConsumptionRate; // в единици на консумация, например mAh/s, когато роботът е активен
    private double maxSpeed; // в метри/сек, максимална скорост
    private double price; // в единици, например USD
    private double failureProbability; // вероятност за неуспех

    private double lidarRange; // в метри, обхват на LiDAR сензора
    private double lidarBatteryConsumptionRate; // в единици на консумация, например mAh/s, когато LiDAR е активен

    public double getBatteryCapacity() {
        return batteryCapacity;
    }

    public void setBatteryCapacity(double batteryCapacity) {
        this.batteryCapacity = batteryCapacity;
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