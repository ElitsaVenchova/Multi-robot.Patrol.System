package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;

//Патрулиране в кръг.Приложимо само за периметър, на който двата края са свързани.
public class RingPatrol implements PatrolModel {

    public RingPatrol(PatrolConfig config) { }

    @Override
    public void initialize(Bot bot, World world) {
        // assign directions (even/odd)
    }

    @Override
    public Position execute(Bot bot, World world, int currentTime) {
        PatrolModel.scan(bot, world, currentTime);

        return null;
    }

    @Override
    public PatrolSection getPatrolSection() { return null; }
}