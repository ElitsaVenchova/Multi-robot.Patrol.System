package bg.uni.sofia.fmi.simulator.domain;

/** Battery energy in Wh. Throughput records actual energy, including partial terminal operations. */
public class Battery {
    private final double capacity; //капацитет на батерията
    private final double averageBatteryLife; // Среден живот на батерията в часове.
    private double currentLevel;//текущо ниво на батерията
    private Double cycleLife, replacementCostEur;

    private double chargedWh, dischargedWh;//Статистика заредена/консумирана енергия

    public Battery(double capacity, double averageBatteryLife, Double life, Double price) {
        if (!Double.isFinite(capacity) || capacity <= 0) throw new IllegalArgumentException("Battery capacity must be positive Wh");
        if ((life == null) != (price == null)) throw new IllegalArgumentException("Supply both cycleLife and replacementCostEur");
        if (life != null && (!Double.isFinite(life) || life <= 0 || !Double.isFinite(price) || price < 0))
            throw new IllegalArgumentException("Invalid battery cycle life/replacement price");
        this.capacity = capacity;
        this.averageBatteryLife = averageBatteryLife;
        this.currentLevel = capacity;
        cycleLife = life;
        replacementCostEur = price;
    }
    public void consume(double amountWh) {
        validate(amountWh);
        double actual = Math.min(currentLevel, amountWh);
        currentLevel = Math.max(0, currentLevel - amountWh);
        addStatsDischarged(actual);
    }
    public void charge(double amountWh) {
        validate(amountWh);
        double actual = Math.min(capacity - currentLevel, amountWh);
        currentLevel = Math.min(capacity, currentLevel + amountWh);
        addStatsCharged(actual);
    }
    private static void validate(double v) { //Валидация на подадения заряд за консумация/зареждане
        if (!Double.isFinite(v) || v < 0) throw new IllegalArgumentException("Energy must be finite non-negative Wh");
    }
    private void addStatsCharged(double amountWh) { chargedWh += amountWh; }
    private void addStatsDischarged(double amountWh) { dischargedWh += amountWh; }
    public boolean isEmpty() { return currentLevel <= 0; }
    public boolean isFull() { return currentLevel >= capacity; }
    public double getCurrentLevel() { return currentLevel; }
    public double getCapacity() { return capacity; }
    public double getAverageBatteryLife() { return averageBatteryLife; }
    public double getChargedWh() { return chargedWh; }
    public double getDischargedWh() { return dischargedWh; }
    public double getEquivalentDischargeCycles() { return dischargedWh / capacity; }
    public double getWearCostEur() { return cycleLife == null ? Double.NaN : getEquivalentDischargeCycles() / cycleLife * replacementCostEur; }
    @Override public String toString() { return String.format(" Battery{capacity=%.2f, currentLevel=%.2f}", capacity, currentLevel); }
}
