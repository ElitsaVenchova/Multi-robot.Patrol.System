package bg.uni.sofia.fmi.simulator.strategy.attack;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

public class PoissonAttack implements LoadModel {

    private double lambda;

    public PoissonAttack(double lambda) {
        this.lambda = lambda;
    }

    @Override
    public List<Attack> generateAttacks(World world, int currentTime) {
        List<Attack> attacks = new ArrayList<>();

        if (RandomProvider.nextDouble() < lambda) {
            double position = RandomProvider.nextDouble() * world.getPerimeterSize();
            attacks.add(new Attack(new Position(position), currentTime)); // ✅ FIX
        }

        return attacks;
    }
}
