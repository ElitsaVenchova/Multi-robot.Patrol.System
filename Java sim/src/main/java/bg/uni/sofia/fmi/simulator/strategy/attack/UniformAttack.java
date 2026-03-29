package bg.uni.sofia.fmi.simulator.strategy.attack;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;

public class UniformAttack implements LoadModel {

    private Random random = new Random();

    @Override
    public List<Attack> generateAttacks(World world, int currentTime) {
        List<Attack> attacks = new ArrayList<>();

        double position = random.nextDouble() * world.getPerimeterSize();
        attacks.add(new Attack(new Position(position), currentTime)); // ✅ FIX

        return attacks;
    }
}
