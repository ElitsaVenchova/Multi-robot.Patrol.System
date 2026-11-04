package bg.uni.sofia.fmi.simulator.domain;

import java.util.List;

/**
 * LiDAR сензор на робота, който може да открива атаки
 * в определен радиус и консумира батерия при използване
 * @param range                  в метри, обхват на LiDAR сензора
 * @param batteryConsumptionRate консумирана енергия на сканиране, в Wh
 */
public record Lidar(double range, double batteryConsumptionRate) {
    /**
     * Numerical robot-range abstraction, without occlusion, field of view or object recognition.
     * Observations reuse the tick's patrol scan and do not consume a second scan charge.
     */
    public boolean canDetectRobot(double perimeterDistance) {
        return canDetectRobot(range, perimeterDistance);
    }

    /** Shared predicate for immutable sensor-range snapshots. Negative range means unavailable. */
    public static boolean canDetectRobot(double detectionRange, double perimeterDistance) {
        return Double.isFinite(detectionRange) && detectionRange >= 0
                && Double.isFinite(perimeterDistance) && perimeterDistance >= 0
                && perimeterDistance <= detectionRange;
    }

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
