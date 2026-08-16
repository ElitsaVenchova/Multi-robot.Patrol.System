package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;

//Патрулиране по секции с противоположни фази за всеки 2 съседни робота
public class CounterPhasePatrol implements PatrolModel {
    private PatrolSection patrolSection;//секцията, в която ще патрулира робота

    public CounterPhasePatrol(PatrolConfig config) { }

    @Override
    public void initialize(Bot bot, World world) {
        // assign directions (even/odd)
    }

    @Override
    public Position execute(Bot bot, World world, int currentTime) {
        //Ако стратегията няма секция за патрулиране, инициализираме патрулирането
        if(patrolSection == null) {
            initialize(bot, world);
        }

        PatrolModel.scan(bot, world, currentTime);

        return null;
    }
}