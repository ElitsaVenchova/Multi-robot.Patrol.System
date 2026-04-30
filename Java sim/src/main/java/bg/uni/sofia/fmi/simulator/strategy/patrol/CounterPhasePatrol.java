package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;

public class CounterPhasePatrol implements PatrolModel {

    private Integer robotsPerSection;

    public CounterPhasePatrol(PatrolConfig config) {
        this.robotsPerSection = config.getRobotsPerSection();
    }

    @Override
    public void initialize(Bot bot) {
        // assign directions (even/odd)
    }

    @Override
    public void execute(Bot bot) {
        double step = bot.getMaxSpeed();
        double perimeter = bot.getWorld().getPerimeter().getSize();
        double targetX;

        if (bot.getId() % 2 == 0) {
            targetX = bot.getPosition().getX() - step;
        } else {
            targetX = bot.getPosition().getX() + step;
        }
        targetX = Math.max(0, Math.min(targetX, perimeter));
        Position target = new Position(targetX);
        
        bot.getBehavior().getNavigation().moveTowards(bot, target);
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}