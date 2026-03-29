package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;

public class CounterPhasePatrol implements PatrolModel {

    @Override
    public void initialize(List<Bot> bots, World world) {
        // assign directions (even/odd)
    }

    @Override
    public void execute(List<Bot> bots, World world) {
        for (int i = 0; i < bots.size(); i++) {
            Bot bot = bots.get(i);

            if (i % 2 == 0) {
                bot.move(); // forward
            } else {
                bot.getPosition().move(-1); // backward (simplified)
            }
        }
    }
}