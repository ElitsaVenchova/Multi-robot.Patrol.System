package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за патрулирането
public class PatrolConfig {
    private String model; //името на вида патрулиране, което може да се използва за избор на конкретна реализация на патрулирането в симулацията
    private double deviationProbability; // опционално, определя колко силно стратегията ще използва случайността на движениеацията
    private int maxDeviationDuration; // колко най-много ticks да продължи движението в другата посока

    public String getModel() {
        return model;
    }

    public void setModel(String type) {
        this.model = type;
    }

    public double getDeviationProbability() {
        return deviationProbability;
    }

    public void setDeviationProbability(double deviationProbability) {
        this.deviationProbability = deviationProbability;
    }

    public int getMaxDeviationDuration() {
        return maxDeviationDuration;
    }

    public void setMaxDeviationDuration(int maxDeviationDuration) {
        this.maxDeviationDuration = maxDeviationDuration;
    }
}
