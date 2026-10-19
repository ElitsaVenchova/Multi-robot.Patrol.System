package bg.uni.sofia.fmi.simulator.domain;

// Клас, представляващ батерията на робота
public class Battery {
    private final double capacity; // максимален капацитет на батерията
    private final double averageBatteryLife;

    private double currentLevel; // текущо ниво на батерията

    public Battery(double capacity, double averageBatteryLife) {
        this.capacity = capacity;
        this.averageBatteryLife = averageBatteryLife;
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

    public double getCapacity() { return capacity; }

    /** Average battery life in hours, as specified by the robot model. */
    public double getAverageBatteryLife() { return averageBatteryLife; }

    @Override
    public String toString() {
        return String.format(" Battery{capacity=%.2f, currentLevel=%.2f}", capacity, currentLevel);
    }
}
