package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;

public class Navigation {
    private ObstacleAvoidance obstacleAvoidance;

    public Navigation(ObstacleAvoidance obstacleAvoidance) {
        this.obstacleAvoidance = obstacleAvoidance;
    }

    public void patrol(Bot bot, World world) {
        if (bot.getState() != BotState.PATROLLING) {
            return;
        }

        // Избягване на препятствия (засега празно)
        obstacleAvoidance.avoid(bot, world);

        // Просто се движим напред. [TODO] Това трябва да е по-сложно, с някаква логика от PatrolModel
        bot.move();
    }

    // Предвижване към зарядна станция
    public void goToChargingStation(Bot bot, World world) {
        moveTowards(bot, bot.getTargetStation().getLocation());
    }

    // [TODO] Да се разгледа какво прави и дали мясото му е тук.
    public void moveTowards(Bot bot, Position target) {
        Position p = bot.getPosition();

        double dx = target.getX() - p.getX();
        double dy = target.getY() - p.getY();

        double step = bot.getMaxSpeed();

        double length = Math.sqrt(dx * dx + dy * dy);

        if (length == 0)
            return;

        p.setX(p.getX() + step * dx / length);
        p.setY(p.getY() + step * dy / length);
    }
}
