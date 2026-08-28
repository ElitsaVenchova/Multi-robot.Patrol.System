package bg.uni.sofia.fmi.simulator.behavior.navigation;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.behavior.collisionAvoidance.ObstacleAvoidance;
import bg.uni.sofia.fmi.simulator.util.MathUtils;

public class Navigation {
    private final ObstacleAvoidance obstacleAvoidance;

    public Navigation(ObstacleAvoidance obstacleAvoidance) {
        this.obstacleAvoidance = obstacleAvoidance;
    }

    // Основен метод за движение напред, който може да се използва от различни стратегии
    public void moveTowards(Bot bot, World world, Position target) {
        //Първо намираме безопасна траектория към целта
        Position nextStep = obstacleAvoidance.avoid(bot, target);

        //Спрямо новата цел изчисляваме размера на стъпката
        double dx = nextStep.getX() - bot.getPosition().getX();
        double distance = Math.abs(dx);

        if (distance < 0.001) return;

        double step = Math.min(bot.getMaxSpeed(), distance);
        double direction = dx > 0 ? 1 : -1;
        double newX = bot.getPosition().getX() + direction * step;

        // clamp
        newX = MathUtils.clamp(newX, 0, world.getPerimeter().getSize());

        nextStep = new Position(newX);//Позицията на следващата стъпка на робота
        bot.move(nextStep);//Преместване на робота
    }
}
