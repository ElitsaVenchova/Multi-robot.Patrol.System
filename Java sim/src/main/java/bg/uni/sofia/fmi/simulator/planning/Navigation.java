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
        // if(bot.getId() == 5) {
        //     System.out.println("Bot " + bot.getId() + " at " + bot.getPosition() + " state: " + bot.getState());
        //     System.out.println("Bot " + bot.getId() + " battery level: " + bot.getBattery().getCurrentLevel());
        //     System.out.println("Bot " + bot.getId() + " goal position: " + bot.getGoalPosition());
        //     System.out.println("Bot " + bot.getId() + " current station: " + (bot.getCurrentStation() != null ? bot.getCurrentStation().getName() : "None"));
        // }
        moveTowards(bot, bot.getGoalPosition());
    }

    // Основен метод за движение напред, който може да се използва от различни стратегии
    public void moveTowards(Bot bot, Position target) {
        // Избягване на препятствия (засега празно)
        obstacleAvoidance.avoid(bot);

        bot.move(target);
    }
}
