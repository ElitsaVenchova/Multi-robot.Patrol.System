package bg.uni.sofia.fmi.simulator.domain;

import java.util.List;

/**
 * LiDAR сензор на робота, който може да открива атаки
 * в определен радиус и консумира батерия при използване
 * @param range                  в метри, обхват на LiDAR сензора
 * @param batteryConsumptionRate когато LiDAR е активен, например mAh/s
 */
public record Lidar(double range, double batteryConsumptionRate) {
    //Открива атаки в радиус от позицията на робота и ги прекъсва, ако са активни
    public void detect(Position botPosition, Perimeter perimeter, int currentTime) {
        List<Attack> nearby = perimeter.getNearbyAttacks(botPosition.getX(), range);
        for (Attack attack : nearby) {
            synchronized (attack) {
                if (attack.isActive() && attack.isExpired(currentTime)) {
                    attack.miss();
                } else if (attack.isActive()) {
                    attack.intercept(currentTime);
                }
            }
        }
    }
}
