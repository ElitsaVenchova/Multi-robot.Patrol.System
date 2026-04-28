package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;

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
            if (bot.getId() % 2 == 0) {
                bot.setDirection(-1);
                bot.getBehavior().getNavigation().moveTowards(bot); // forward
            } else {
                bot.setDirection(1);
                bot.getBehavior().getNavigation().moveTowards(bot); // backward (simplified)
            }
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}