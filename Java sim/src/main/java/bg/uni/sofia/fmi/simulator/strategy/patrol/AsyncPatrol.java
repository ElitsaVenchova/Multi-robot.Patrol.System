package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;

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
        if (bot.getPosition().getX() > bot.getWorld().getPerimeter().getSize() * 0.9) {
            bot.setDirection(-1);
        }

        if (bot.getPosition().getX() < bot.getWorld().getPerimeter().getSize() * 0.1) {
            bot.setDirection(1);
        }
        bot.getBehavior().getNavigation().moveTowards(bot);
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}