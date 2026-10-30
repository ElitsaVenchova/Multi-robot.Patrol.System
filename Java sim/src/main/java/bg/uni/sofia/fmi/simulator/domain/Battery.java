package bg.uni.sofia.fmi.simulator.domain;

/** Battery energy in Wh. Throughput records actual energy, including partial terminal operations. */
public class Battery {
    private final double capacity;
    private final double averageBatteryLife; // Legacy calendar-life metadata in hours, not cycles.
    private double currentLevel;
    private double chargedWh, dischargedWh;
    private Double cycleLife, replacementCostEur;

    public Battery(double capacity, double averageBatteryLife) {
        if (!Double.isFinite(capacity) || capacity <= 0) throw new IllegalArgumentException("Battery capacity must be positive Wh");
        this.capacity = capacity; this.averageBatteryLife = averageBatteryLife; this.currentLevel = capacity;
    }
    public void setWearModel(Double life, Double price) {
        if ((life == null) != (price == null)) throw new IllegalArgumentException("Supply both cycleLife and replacementCostEur");
        if (life != null && (!Double.isFinite(life) || life <= 0 || !Double.isFinite(price) || price < 0))
            throw new IllegalArgumentException("Invalid battery cycle life/replacement price");
        cycleLife = life; replacementCostEur = price;
    }
    public void consume(double amountWh) {
        validate(amountWh);
        double actual = Math.min(currentLevel, amountWh);
        currentLevel = Math.max(0, currentLevel - amountWh);
        dischargedWh += actual;
    }
    public void charge(double amountWh) {
        validate(amountWh);
        double actual = Math.min(capacity - currentLevel, amountWh);
        currentLevel = Math.min(capacity, currentLevel + amountWh);
        chargedWh += actual;
    }
    private static void validate(double v) {
        if (!Double.isFinite(v) || v < 0) throw new IllegalArgumentException("Energy must be finite non-negative Wh");
    }
    public int estimateTimeToFull(double chargeWhPerTick) { return (int) ((capacity - currentLevel) / chargeWhPerTick); }
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
