package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за моделите на зарядните станции
public class ChargingStationModelConfig {
    private String name; //името на модела на зарядната станция, което може да се използва за избор на конкретна реализация на зарядна станция в симулацията
    private double price; //цена
    private int slots; //брой слотове за зареждане
    private double power; //мощност на зареждане (напр. в kW)
    private double failureProbability; //вероятност за повреда при всяко зареждане, което може да се използва за симулиране на реалистично поведение на зарядните станции

    public String getName() { return name; }
    public void setName(String name) { this.name = name; }

    public double getPrice() { return price; }
    public void setPrice(double price) { this.price = price; }

    public int getSlots() { return slots; }
    public void setSlots(int slots) { this.slots = slots; }

    public double getPower() { return power; }
    public void setPower(double power) { this.power = power; }

    public double getFailureProbability() { return failureProbability; }
    public void setFailureProbability(double failureProbability) {
        this.failureProbability = failureProbability;
    }
}