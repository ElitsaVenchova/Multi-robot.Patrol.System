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

    //[TODO] В описанието на UML диаграмата май имаше по-реалистична консумация, която зависи от разстоянието и скоростта
    public void consume() {
        currentLevel -= consumptionRate;
        if (currentLevel < 0)
            currentLevel = 0;
    }

    //[TODO] В описанието на UML диаграмата май имаше по-реалистично зареждане, който зависи от времето и зарядното устройство
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
