package bg.uni.sofia.fmi.simulator.domain;

public class Lidar {
    private double range;
    private double batteryConsumptionRate;

    public Lidar(double range, double batteryConsumptionRate) {
        this.range = range;
    }

    // [TODO] Може би ще е по-добре да проверява "петиметъра" по обхвата,
    // а не само разстоянието между бота и всяка атака
    public boolean detect(Position botPosition, Attack attack) {
        double dx = Math.abs(botPosition.getX() - attack.getPosition().getX());
        return dx <= this.range;
    }

    public double getBatteryConsumptionRate() {
        return batteryConsumptionRate;
    }

    public double getRange() {
        return range;
    }
}
