package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за моделите на роботите и неговите компоненти
// Distances m, speed m/min, capacity Wh, movement Wh/m, LiDAR Wh/scan, prices EUR.
public class RobotModelConfig {
    private String name;
    private String type;
    private double maxSpeed;
    private double price;
    private double failureProbability;
    private double batteryConsumptionRate;
    private BatteryConfig battery;
    private LidarConfig lidar;
    private CommunicationModuleConfig communicationModule;
    private double baselineConsumptionWhPerMinute = 0.01;

    public String getName() { return name; }
    public void setName(String name) { this.name = name; }
    public String getType() { return type; }
    public void setType(String type) { this.type = type; }
    public double getMaxSpeed() { return maxSpeed; }
    public void setMaxSpeed(double maxSpeed) { this.maxSpeed = maxSpeed; }
    public double getPrice() { return price; }
    public void setPrice(double price) { this.price = price; }
    public double getFailureProbability() { return failureProbability; }
    public void setFailureProbability(double failureProbability) { this.failureProbability = failureProbability; }
    public double getBatteryConsumptionRate() { return batteryConsumptionRate; }
    public void setBatteryConsumptionRate(double batteryConsumptionRate) { this.batteryConsumptionRate = batteryConsumptionRate; }
    public BatteryConfig getBattery() { return battery; }
    public void setBattery(BatteryConfig battery) { this.battery = battery; }
    public LidarConfig getLidar() { return lidar; }
    public void setLidar(LidarConfig lidar) { this.lidar = lidar; }
    public CommunicationModuleConfig getCommunicationModule() { return communicationModule; }
    public void setCommunicationModule(CommunicationModuleConfig communicationModule) { this.communicationModule = communicationModule; }
    public double getBaselineConsumptionWhPerMinute() { return baselineConsumptionWhPerMinute; }
    public void setBaselineConsumptionWhPerMinute(double v) { baselineConsumptionWhPerMinute = v; }

    public static class BatteryConfig {
        private Double cycleLife;
        private Double replacementCostEur;
        public Double getCycleLife() { return cycleLife; }
        public void setCycleLife(Double v) { cycleLife = v; }
        public Double getReplacementCostEur() { return replacementCostEur; }
        public void setReplacementCostEur(Double v) { replacementCostEur = v; }
        private double batteryCapacity;
        private double averageBatteryLife;
        public double getBatteryCapacity() { return batteryCapacity; }
        public void setBatteryCapacity(double batteryCapacity) { this.batteryCapacity = batteryCapacity; }
        public double getAverageBatteryLife() { return averageBatteryLife; }
        public void setAverageBatteryLife(double averageBatteryLife) { this.averageBatteryLife = averageBatteryLife; }
    }

    public static class LidarConfig {
        private double lidarRange;
        private double lidarBatteryConsumptionRate;
        public double getLidarRange() { return lidarRange; }
        public void setLidarRange(double lidarRange) { this.lidarRange = lidarRange; }
        public double getLidarBatteryConsumptionRate() { return lidarBatteryConsumptionRate; }
        public void setLidarBatteryConsumptionRate(double rate) { this.lidarBatteryConsumptionRate = rate; }
    }

    public static class CommunicationModuleConfig {
        private double communicationRange;
        private double speed;
        private double latency;
        private double reliability;
        public double getCommunicationRange() { return communicationRange; }
        public void setCommunicationRange(double communicationRange) { this.communicationRange = communicationRange; }
        public double getSpeed() { return speed; }
        public void setSpeed(double speed) { this.speed = speed; }
        public double getLatency() { return latency; }
        public void setLatency(double latency) { this.latency = latency; }
        public double getReliability() { return reliability; }
        public void setReliability(double reliability) { this.reliability = reliability; }
    }
}
