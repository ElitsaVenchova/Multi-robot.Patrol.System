package bg.uni.sofia.fmi.simulator.domain;

import java.util.List;

/**
 * @param range                  в метри, обхват на LiDAR сензора
 * @param batteryConsumptionRate когато LiDAR е активен, например mAh/s
 */ // LiDAR сензор на робота, който може да открива атаки
// в определен радиус и консумира батерия при използване
public record Lidar(double range, double batteryConsumptionRate) {
    //Открива атаки в радиус от позицията на робота и ги прекъсва, ако са активни
    public void detect(Position botPosition, Perimeter perimeter, int currentTime) {
        List<Attack> nearby = perimeter.getNearbyAttacks(botPosition.getX(), range);
        for (Attack attack : nearby) {
            synchronized (attack) {
                if (attack.isActive() && attack.isExpired(currentTime)) {
                    attack.intercept(currentTime);
                } else if (attack.isActive()) {
                    attack.intercept(currentTime);
                }
            }
        }
    }
}
