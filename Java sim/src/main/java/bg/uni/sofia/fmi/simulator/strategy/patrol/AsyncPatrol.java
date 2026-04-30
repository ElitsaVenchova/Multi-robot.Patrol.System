package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;

public class AsyncPatrol implements PatrolModel {

    private Integer robotsPerSection;

    public AsyncPatrol(PatrolConfig config) {
        this.robotsPerSection = config.getRobotsPerSection();
    }
  
    @Override
    public void initialize(Bot bot) {
        // optional random offsets
    }

    @Override
    public void execute(Bot bot) {
        double perimeter = bot.getWorld().getPerimeter().getSize();
        double randomOffset = (Math.random() - 0.5) * 10; // configurable
        double targetX = bot.getPosition().getX() + randomOffset;
        targetX = Math.max(0, Math.min(targetX, perimeter));
        Position target = new Position(targetX);

        bot.getBehavior().getNavigation().moveTowards(bot, target); // move towards the target with obstacle avoidance
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}