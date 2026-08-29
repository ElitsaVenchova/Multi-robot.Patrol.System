package bg.uni.sofia.fmi.simulator.strategy.attack;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.World;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

//Поасоново разпределени атаки по време, а по периметъра са разпределени равномерно.
// Използва се за базов случай за анализ в симулацията
public class PoissonAttack implements LoadModel {

    private final double lambda;
    private final Integer duration;

    public PoissonAttack(double lambda, Integer duration) {
        this.lambda = lambda;
        this.duration = duration;
    }

    @Override
    public List<Attack> generateAttacks(World world, int currentTime) {
        List<Attack> attacks = new ArrayList<>();

        if (RandomProvider.nextDouble() < lambda) {
            double position = RandomProvider.nextDouble() * world.getPerimeter().getSize();
            attacks.add(new Attack(new Position(position), currentTime, duration));
        }
        return attacks;
    }
}
