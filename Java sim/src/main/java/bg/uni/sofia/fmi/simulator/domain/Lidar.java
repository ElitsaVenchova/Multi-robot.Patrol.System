package bg.uni.sofia.fmi.simulator.domain;

import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

// LiDAR сензор на робота, който може да открива атаки 
// в определен радиус и консумира батерия при използване
public class Lidar {
    private double range; // в метри, обхват на LiDAR сензора
    private double batteryConsumptionRate; // когато LiDAR е активен, например mAh/s

    public Lidar(double range, double batteryConsumptionRate) {
        this.range = range;
        this.batteryConsumptionRate = batteryConsumptionRate;
    }

    public double getBatteryConsumptionRate() {
        return batteryConsumptionRate;
    }

    public double getRange() {
        return range;
    }

    //Открива атаки в радиус от позицията на робота и ги прекъсва, ако са активни
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
