package bg.uni.sofia.fmi.simulator.domain;

public class Battery {
    private double capacity;
    private double currentLevel;
    private double consumptionRate;

    public Battery(double capacity, double consumptionRate) {
        this.capacity = capacity;
        this.currentLevel = capacity;
        this.consumptionRate = consumptionRate;
    }

    public void consume() {
        currentLevel -= consumptionRate;
        if (currentLevel < 0)
            currentLevel = 0;
    }

    public void recharge(double amount) {
        currentLevel += amount;
        if (currentLevel > capacity)
            currentLevel = capacity;
    }

    public boolean isEmpty() {
        return currentLevel <= 0;
    }

    public double getCurrentLevel() {
        return currentLevel;
    }
}
