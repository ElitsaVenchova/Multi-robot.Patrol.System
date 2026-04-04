package bg.uni.sofia.fmi.simulator.config;

public class ChargingStationModelConfig {

    private String name;
    private double price;
    private int slots;
    private double power;
    private double failureProbability;

    // Getters & setters

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