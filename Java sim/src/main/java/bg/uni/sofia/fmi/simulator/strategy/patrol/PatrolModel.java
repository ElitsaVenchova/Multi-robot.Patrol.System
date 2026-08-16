package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Bot;

public interface PatrolModel {
    //setup (positions, phases)
    void initialize(Bot bot);
    //called every tick
    void execute(Bot bot);
}
