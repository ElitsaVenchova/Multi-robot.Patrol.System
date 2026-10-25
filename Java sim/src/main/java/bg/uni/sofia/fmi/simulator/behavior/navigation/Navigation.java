package bg.uni.sofia.fmi.simulator.behavior.navigation;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.behavior.collisionAvoidance.ObstacleAvoidance;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;
import bg.uni.sofia.fmi.simulator.util.MathUtils;
import bg.uni.sofia.fmi.simulator.strategy.patrol.Direction;

public class Navigation {
    private final ObstacleAvoidance obstacleAvoidance;

    public Navigation(ObstacleAvoidance obstacleAvoidance) {
        this.obstacleAvoidance = obstacleAvoidance;
    }

    // Основен метод за движение напред, който може да се използва от различни стратегии
    public void moveTowards(Bot bot, Position target) {
        moveTowards(bot, target, null);
    }

    /** An explicit direction preserves the requested circular arc instead of taking a shortcut. */
    public void moveTowards(Bot bot, Position target, Direction requestedDirection) {
        //Първо намираме безопасна траектория към целта
        Position nextStep = obstacleAvoidance.avoid(bot, target);

        double currentX = bot.getPosition().getX();
        double perimeterLength = World.getPerimeter().getSize();
        double distance;
        int direction;

        if (World.getPerimeter().getType() == PerimeterType.CIRCULAR) {
            if (requestedDirection != null) {
                int sign = requestedDirection == Direction.RIGHT ? 1 : -1;
                double directedDistance = sign * (nextStep.getX() - currentX);
                distance = directedDistance >= 0 ? directedDistance
                        : GeometryUtils.normalizePosition(directedDistance);
                direction = requestedDirection == Direction.RIGHT ? 1 : -1;
            } else {
                distance = GeometryUtils.ringDistance(currentX, nextStep.getX());
                direction = GeometryUtils.shortestRingDirection(currentX, nextStep.getX());
            }
        } else {
            distance = Math.abs(nextStep.getX() - currentX);
            direction = nextStep.getX() > currentX ? 1 : -1;
        }
        if (distance < 0.001) return;

        double step = Math.min(bot.getMaxSpeed(), distance);
        double newX = currentX + direction * step;
        newX = World.getPerimeter().getType() == PerimeterType.CIRCULAR
                ? GeometryUtils.normalizePosition(newX)
                : MathUtils.clamp(newX, 0, perimeterLength);
        if (requestedDirection != null && World.getPerimeter().getType() == PerimeterType.CIRCULAR) {
            bot.move(new Position(newX), step);
        } else {
            bot.move(new Position(newX));
        }
    }
}