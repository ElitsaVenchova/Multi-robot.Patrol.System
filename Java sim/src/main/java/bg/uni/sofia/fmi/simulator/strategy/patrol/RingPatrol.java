package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;

public class RingPatrol implements PatrolModel {
    private Integer robotsPerSection;

    public RingPatrol(PatrolConfig config) {
        this.robotsPerSection = config.getRobotsPerSection();
    }

    @Override
    public void initialize(Bot bot) {
        // Optionally distribute bots evenly
    }

    @Override
    public void execute(Bot bot) {
        bot.getBehavior().getNavigation().moveTowards(bot); // simple continuous movement
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
    
}
