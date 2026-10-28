package bg.uni.sofia.fmi.simulator.experiments;

import bg.uni.sofia.fmi.simulator.domain.*;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.strategy.attack.PoissonAttack;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;
import org.junit.Test;
import java.util.*;
import static org.junit.Assert.*;

public class AttackReproducibilityTest {
    private List<String> scenario(long seed, double probability) {
        World.reset(); World.setPerimeter(new Perimeter(100, PerimeterType.LINEAR)); RandomProvider.setSeed(seed);
        var model = new PoissonAttack(probability, 10);
        List<String> attacks = new ArrayList<>();
        for (int t = 0; t < 200; t++) for (var attack : model.generateAttacks(t))
            attacks.add(t + ":" + attack.getPosition().getX());
        return attacks;
    }
    @Test public void recordedSeedReproducesScenarioAndDifferentSeedChangesIt() {
        assertEquals(scenario(42, 0.5), scenario(42, 0.5)); assertNotEquals(scenario(42, 0.5), scenario(43, 0.5));
    }
    @Test public void probabilityControlsGeneratedIntensity() {
        assertEquals(0, scenario(42, 0).size()); assertEquals(200, scenario(42, 1).size());
        assertTrue(scenario(42, 0.1).size() < scenario(42, 0.8).size());
    }
}
