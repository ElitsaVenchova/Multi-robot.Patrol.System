package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;

public class Navigation {
    private ObstacleAvoidance obstacleAvoidance;

    public Navigation(ObstacleAvoidance obstacleAvoidance) {
        this.obstacleAvoidance = obstacleAvoidance;
    }

    // Предвижване към зарядна станция
    public void goToChargingStation(Bot bot) {
        moveTowards(bot, bot.getGoalPosition());
    }

    // Основен метод за движение напред, който може да се използва от различни стратегии
    public void moveTowards(Bot bot, Position target) {
        // Избягване на препятствия (засега празно)
        obstacleAvoidance.avoid(bot);

        bot.move(target);
    }
}
