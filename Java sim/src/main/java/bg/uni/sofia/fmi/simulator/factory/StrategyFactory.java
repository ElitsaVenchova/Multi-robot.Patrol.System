package bg.uni.sofia.fmi.simulator.factory;

import bg.uni.sofia.fmi.simulator.config.AttackConfig;
import bg.uni.sofia.fmi.simulator.config.PatrolConfig;
import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;
import bg.uni.sofia.fmi.simulator.strategy.attack.PoissonAttack;
import bg.uni.sofia.fmi.simulator.strategy.attack.UniformAttack;
import bg.uni.sofia.fmi.simulator.strategy.attack.VulnerabilityAttack;
import bg.uni.sofia.fmi.simulator.strategy.patrol.AsyncPatrol;
import bg.uni.sofia.fmi.simulator.strategy.patrol.CounterPhasePatrol;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolModel;
import bg.uni.sofia.fmi.simulator.strategy.patrol.RingPatrol;

// Фабрика за създаване на стратегии за патрулиране и генериране на атаки от конфигурацията
public class StrategyFactory {

    // Създаване на модел за патрулиране от конфигурацията. 
    // В зависимост от зададения тип, се създава съответната стратегия.
    public static PatrolModel createPatrol(PatrolConfig config) {
        if (config == null || config.getModel() == null) {
            throw new IllegalArgumentException("Patrol config is invalid");
        }
        // В зависимост от зададения тип, се създава съответната стратегия. 
        // Ако типът е невалиден, се хвърля грешка.
        switch (config.getModel()) {
            case "RingPatrol":
                return new RingPatrol(config);
            case "CounterPhasePatrol":
                return new CounterPhasePatrol(config);
            case "AsyncPatrol":
                return new AsyncPatrol(config);
            default:
                throw new IllegalArgumentException(
                        "Unknown patrol model: " + config.getModel());
        }
    }

    // Създаване на модел за генериране на атаки от конфигурацията.
    // В зависимост от зададения тип, се създава съответната стратегия.
    public static LoadModel createAttack(AttackConfig config) {
        if (config == null || config.getModel() == null) {
            throw new IllegalArgumentException("Attack config is invalid");
        }
        // В зависимост от зададения тип, се създава съответната стратегия.
        // Ако типът е невалиден, се хвърля грешка.
        switch (config.getModel()) {
            case "PoissonAttack":
                if (config.getLambda() == null) {
                    throw new IllegalArgumentException("PoissonAttack requires lambda");
                }
                return new PoissonAttack(config.getLambda(),
                        config.getDuration());
            case "UniformAttack":
                return new UniformAttack(config.getDuration());
            case "VulnerabilityAttack":
                return new VulnerabilityAttack(config.getDuration());
            default:
                throw new IllegalArgumentException("Unknown attack model: " + config.getModel());
        }
    }
}