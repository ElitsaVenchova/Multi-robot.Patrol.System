package bg.uni.sofia.fmi.simulator.domain;

import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

public class Lidar {
    private double range;
    private double batteryConsumptionRate;

    public Lidar(double range, double batteryConsumptionRate) {
        this.range = range;
    }

    public double getBatteryConsumptionRate() {
        return batteryConsumptionRate;
    }

    public double getRange() {
        return range;
    }

    public void detect(Position botPosition, Perimeter perimeter, int currentTime) {
        List<Attack> nearby = perimeter.getNearbyAttacks(botPosition.getX(), range);

        for (Attack attack : nearby) {
            synchronized (attack) {
                if (attack.getStatus() == AttackStatus.ACTIVE) {
                    attack.intercept(currentTime);
                }
            }
        }
    }
}
