package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;

//Патрулиране по секции като всеко робот се вдижи независимо в своята секция
public class AsyncPatrol implements PatrolModel {

  
    @Override
    public void initialize(Bot bot, World world) {
        // optional random offsets
    }

    @Override
    public void execute(Bot bot, World world, int currentTime) {
        double perimeter = world.getPerimeter().getSize();
        double randomOffset = (Math.random() - 0.5) * 10; // configurable
        double targetX = bot.getPosition().getX() + randomOffset;
        targetX = Math.max(0, Math.min(targetX, perimeter));
        Position target = new Position(targetX);

        bot.getBehavior().getNavigation().moveTowards(bot, target); // move towards the target with obstacle avoidance

        PatrolModel.scan(bot, world, currentTime);
    }
}