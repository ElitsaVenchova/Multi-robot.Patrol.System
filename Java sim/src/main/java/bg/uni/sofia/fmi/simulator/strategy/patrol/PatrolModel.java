package bg.uni.sofia.fmi.simulator.strategy.patrol;

import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.World;

public interface PatrolModel {
    //setup (positions, phases)
    void initialize(List<Bot> bots, World world);
    //called every tick
    void execute(List<Bot> bots, World world);
}
