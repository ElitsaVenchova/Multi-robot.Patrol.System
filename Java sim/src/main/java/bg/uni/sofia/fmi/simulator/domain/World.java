package bg.uni.sofia.fmi.simulator.domain;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

public class World {

    private List<Bot> bots = new ArrayList<>();
    private List<Attack> attacks = new ArrayList<>();
    private double perimeterSize;

    public World(double perimeterSize) {
        this.perimeterSize = perimeterSize;
    }

    public void addBot(Bot bot) {
        bots.add(bot);
    }

    public void addAttack(Attack attack) {
        attacks.add(attack);
    }

    public void tick(int currentTime) {
        // Move robots
        for (Bot bot : bots) {
            bot.move();
        }

        // [TODO] Detection да се гледа по периметъра спрямо обхвата, а не развнение всеки със всеки
        for (Bot bot : bots) {
            for (Attack attack : attacks) {
                if (attack.getStatus() == AttackStatus.ACTIVE
                        && bot.detect(attack)) {
                    attack.intercept(currentTime); // ✅
                }
            }
        }

        // [TODO] Два вида "пропуснати" атаки:
        // 1. Отчитане на пропуснати след определено време
        // 2. Отчитане на време за засичане
        for (Attack attack : attacks) {
            if (attack.getStatus() == AttackStatus.ACTIVE) {

                // example rule: if not detected after X time → missed
                int lifetime = currentTime - attack.getCreationTime();

                if (lifetime > 50) { // [TODO] threshold (configurable later)
                    attack.miss();
                }
            }
        }
    }

    public List<Attack> getAttacks() {
        return attacks;
    }

    public double getPerimeterSize() {
        return perimeterSize;
    }

    public List<Bot> getBots() {
        return bots;
    }
}
