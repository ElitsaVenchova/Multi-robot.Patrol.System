package bg.uni.sofia.fmi.simulator.factory;

import bg.uni.sofia.fmi.simulator.config.AttackConfig;
import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;
import bg.uni.sofia.fmi.simulator.strategy.attack.PoissonAttack;
import bg.uni.sofia.fmi.simulator.strategy.attack.UniformAttack;
import bg.uni.sofia.fmi.simulator.strategy.attack.VulnerabilityAttack;
import bg.uni.sofia.fmi.simulator.strategy.patrol.AsyncPatrol;
import bg.uni.sofia.fmi.simulator.strategy.patrol.CounterPhasePatrol;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import bg.uni.sofia.fmi.simulator.strategy.patrol.RingPatrol;

/**
 * mapping DSL strings → strategy objects
 * injecting parameters (like lambda)
 */
public class StrategyFactory {

    // ===== PATROL =====
    public static PatrolModel createPatrol(String patrolModel) {

        if (patrolModel == null) {
            throw new IllegalArgumentException("Patrol model is null");
        }

        switch (patrolModel) {
            case "RingPatrol":
                return new RingPatrol();

            case "CounterPhasePatrol":
                return new CounterPhasePatrol();

            case "AsyncPatrol":
                return new AsyncPatrol();

            default:
                throw new IllegalArgumentException("Unknown patrol model: " + patrolModel);
        }
    }

    // ===== ATTACK =====
    public static LoadModel createAttack(AttackConfig config) {

        if (config == null || config.getType() == null) {
            throw new IllegalArgumentException("Attack config is invalid");
        }

        switch (config.getType()) {
            case "PoissonAttack":
                if (config.getLambda() == null) {
                    throw new IllegalArgumentException("PoissonAttack requires lambda");
                }
                return new PoissonAttack(config.getLambda());

            case "UniformAttack":
                return new UniformAttack();

            case "VulnerabilityAttack":
                return new VulnerabilityAttack();

            default:
                throw new IllegalArgumentException("Unknown attack model: " + config.getType());
        }
    }
}