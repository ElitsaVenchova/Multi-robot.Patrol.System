package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за моделите на роботите и неговите компоненти
// Distances m, speed m/min, capacity Wh, movement Wh/m, LiDAR Wh/scan, prices EUR.
public class RobotModelConfig {
    private String name;//име на робота
    private String type; //тип на робота дрон/наземен
    private double maxSpeed; //максимална скорост на робота
    private double price; //цена
    private double failureProbability; //вероятност за отказ
    private double batteryConsumptionRate; //потребление на енергия при движение
    private double baselineConsumptionWhPerMinute; //базово потребление на енергия на робота
    //Кофигурации на модулите на робота
    private BatteryConfig battery; //конфигурация на батерията
    private LidarConfig lidar; //конфигурация на лазерния модул
    private CommunicationModuleConfig communicationModule; //конфигурация на модула за комуникация

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
        private Double cycleLife; //жизнен цикъл на батерията
        private Double replacementCostEur; //цена на замяна на батерията
        private double batteryCapacity; //капацитет на батерията
        private double averageBatteryLife; //средна жизнен цикъл на батерията
        public Double getCycleLife() { return cycleLife; }
        public void setCycleLife(Double v) { cycleLife = v; }
        public Double getReplacementCostEur() { return replacementCostEur; }
        public void setReplacementCostEur(Double v) { replacementCostEur = v; }
        public double getBatteryCapacity() { return batteryCapacity; }
        public void setBatteryCapacity(double batteryCapacity) { this.batteryCapacity = batteryCapacity; }
        public double getAverageBatteryLife() { return averageBatteryLife; }
        public void setAverageBatteryLife(double averageBatteryLife) { this.averageBatteryLife = averageBatteryLife; }
    }

    public static class LidarConfig {
        private double lidarRange; //дистанция на измерване на лазерния модул
        private double lidarBatteryConsumptionRate; //потребление на енергия при измерване
        public double getLidarRange() { return lidarRange; }
        public void setLidarRange(double lidarRange) { this.lidarRange = lidarRange; }
        public double getLidarBatteryConsumptionRate() { return lidarBatteryConsumptionRate; }
        public void setLidarBatteryConsumptionRate(double rate) { this.lidarBatteryConsumptionRate = rate; }
    }

    public static class CommunicationModuleConfig {
        private double communicationRange; //обхват на модула за комуникация
        private double speed; //скорост на обмена на данни
        private double latency; //време за изпращане на данни
        private double reliability; //надеждност на модула за комуникация
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
