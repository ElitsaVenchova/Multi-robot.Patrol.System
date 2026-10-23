package bg.uni.sofia.fmi.simulator.config;

import bg.uni.sofia.fmi.simulator.domain.enums.EncounterRearmPolicy;

// Конфигурация за патрулирането
public class PatrolConfig {

    private String model; //името на вида патрулиране, което може да се използва за избор на конкретна реализация на патрулирането в симулацията
    private double deviationProbability; // опционално, определя колко силно стратегията ще използва случайността на движениеацията
    private int maxDeviationDuration; // колко най-много ticks да продължи движението в другата посока
    private EncounterRearmPolicy encounterRearmPolicy = EncounterRearmPolicy.EXIT_EITHER_RANGE; // опционално, определя как се обработват срещи м/у роботи за EncounterPatrol

    public String getModel() {
        return model;
    }
    public void setModel(String type) {
        this.model = type;
    }

    public double getDeviationProbability() {
        return deviationProbability;
    }
    public void setDeviationProbability(double deviationProbability) { this.deviationProbability = deviationProbability; }

    public int getMaxDeviationDuration() {
        return maxDeviationDuration;
    }
    public void setMaxDeviationDuration(int maxDeviationDuration) {
        this.maxDeviationDuration = maxDeviationDuration;
    }

    public EncounterRearmPolicy getEncounterRearmPolicy() {
        return encounterRearmPolicy;
    }
    public void setEncounterRearmPolicy(EncounterRearmPolicy encounterRearmPolicy) { this.encounterRearmPolicy = encounterRearmPolicy; }
}
