package bg.uni.sofia.fmi.simulator.strategy.attack;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

public class UniformAttack implements LoadModel {

    private final double probability;
    private final Integer duration;

    public UniformAttack(double probability, Integer duration) {
        validateProbability(probability);

        this.probability = probability;
        this.duration = duration;
    }

    @Override
    public List<Attack> generateAttacks(World world, int currentTime) {
        List<Attack> attacks = new ArrayList<>();

        if (RandomProvider.nextDouble() >= probability) {
            return attacks;
        }

        double position = RandomProvider.nextDouble() * world.getPerimeter().getSize();
        attacks.add(new Attack(new Position(position), currentTime, duration));

        return attacks;
    }

    private void validateProbability(double probability) {
        if (probability < 0.0 || probability > 1.0) {
            throw new IllegalArgumentException("UniformAttack probability must be between 0.0 and 1.0");
        }
    }
}
