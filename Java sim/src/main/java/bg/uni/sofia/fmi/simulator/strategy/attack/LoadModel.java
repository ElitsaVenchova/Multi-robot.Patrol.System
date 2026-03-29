package bg.uni.sofia.fmi.simulator.strategy.attack;

import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.World;

public interface LoadModel {
    List<Attack> generateAttacks(World world, int currentTime);
}
