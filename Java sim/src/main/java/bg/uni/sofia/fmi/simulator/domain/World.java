package bg.uni.sofia.fmi.simulator.domain;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

public class World {

    private List<Bot> bots = new ArrayList<>();
    private List<Attack> attacks = new ArrayList<>();
    private List<ChargingStation> chargingStations;
    private Perimeter perimeter;

    public World(double perimeter) {
        this.perimeter = new Perimeter((int) perimeter);
    }

    public void addBots(List<Bot> bots) {
        this.bots.addAll(bots);
    }

    public void addAttack(Attack attack) {
        attacks.add(attack);
    }

    public void addChargingStations(List<ChargingStation> chargingStations) {
        this.chargingStations = chargingStations;
    }

    public void tick(int currentTime) {
        // Move robots
        for (Bot bot : bots) {
            bot.move();
        }

        // [TODO] Detection да се гледа по периметъра спрямо обхвата, а не сравнение
        // всеки със всеки
        for (Bot bot : bots) {
            List<Attack> nearby = perimeter.getNearbyAttacks(
                    bot.getPosition().getX(),
                    bot.getLidar().getRange());

            for (Attack attack : nearby) {
                if (attack.isActive()) {
                    attack.intercept(currentTime);
                }
            }
        }

        // 1. Отчитане на пропуснати след определено време
        // 2. Отчитане на време за засичане
        for (Attack attack : attacks) {
            if (attack.getStatus() == AttackStatus.ACTIVE) {
                // Expiration
                if (attack.isExpired(currentTime)) {
                    attack.miss();
                    continue;
                }
            }
        }
    }

    public List<Attack> getAttacks() {
        return attacks;
    }

    public Perimeter getPerimeter() {
        return perimeter;
    }

    public List<Bot> getBots() {
        return bots;
    }

    public List<ChargingStation> getChargingStations() {
        return chargingStations;
    }
}
