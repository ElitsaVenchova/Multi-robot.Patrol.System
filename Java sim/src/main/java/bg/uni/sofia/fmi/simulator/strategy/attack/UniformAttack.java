package bg.uni.sofia.fmi.simulator.strategy.attack;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

public class UniformAttack implements LoadModel {

    private Random random = RandomProvider.getRandom();
    private Integer duration;

    public UniformAttack(Integer duration) {
        this.duration = duration;
    }

    @Override
    public List<Attack> generateAttacks(World world, int currentTime) {
        List<Attack> attacks = new ArrayList<>();

        double position = random.nextDouble() * world.getPerimeter().getSize();
        attacks.add(new Attack(new Position(position), currentTime, duration));

        return attacks;
    }
}
