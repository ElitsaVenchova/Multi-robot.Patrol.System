package bg.uni.sofia.fmi.simulator.domain;

public class Battery {
    private double capacity;
    private double currentLevel;

    public Battery(double capacity) {
        this.capacity = capacity;
        this.currentLevel = capacity;
    }

    //[TODO] Да се направи по-реалистична консумация
    public void consume(double consumptionRate) {
        currentLevel -= consumptionRate;
        if (currentLevel < 0)
            currentLevel = 0;
    }

    //[TODO] Да се направи по-реалистично зареждане
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
