package bg.uni.sofia.fmi.simulator.strategy.patrol;

import bg.uni.sofia.fmi.simulator.domain.Bot;

public interface PatrolModel {
    //setup (positions, phases)
    void initialize(Bot bot);
    //called every tick
    // [TODO] Да се оправи във всяка реализация
    void execute(Bot bot);
}
