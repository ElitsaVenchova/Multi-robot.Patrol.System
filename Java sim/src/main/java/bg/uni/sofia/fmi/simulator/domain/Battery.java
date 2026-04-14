package bg.uni.sofia.fmi.simulator.domain;

// Клас, представляващ батерията на робота
public class Battery {
    private double capacity; // максимален капацитет на батерията

    private double currentLevel; // текущо ниво на батерията

    public Battery(double capacity) {
        this.capacity = capacity;
        this.currentLevel = capacity;
    }

    // [TODO] Да се направи по-реалистична консумация
    public void consume(double consumptionRate) {
        currentLevel -= consumptionRate;
        if (currentLevel < 0)
            currentLevel = 0;
    }

    // [TODO] Да се направи по-реалистично зареждане
    public void charge(double amount) {
        currentLevel += amount;
        if (currentLevel > capacity)
            currentLevel = capacity;
    }

    public int estimateTimeToFull(double chargeRate) {
        return (int) ((capacity - currentLevel) / chargeRate);
    }

    public boolean isEmpty() {
        return currentLevel <= 0;
    }

    public boolean isFull() {
        return currentLevel >= capacity;
    }

    public double getCurrentLevel() {
        return currentLevel;
    }
}
