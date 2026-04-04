package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.ChargingStationStatus;

public class ChargingStation {

    private String name;
    private double price;
    private int totalSlots;
    private double power;
    private double failureProbability;

    private int occupiedSlots;
    private Position location;
    private ChargingStationStatus status;

    public ChargingStation(String name,
                           double price,
                           int totalSlots,
                           double power,
                           double failureProbability,
                           Position location) {

        this.name = name;
        this.price = price;
        this.totalSlots = totalSlots;
        this.power = power;
        this.failureProbability = failureProbability;
        this.location = location;

        this.occupiedSlots = 0;
        this.status = ChargingStationStatus.AVAILABLE;
    }

    public void updateStatus() {
        if (occupiedSlots >= totalSlots) {
            status = ChargingStationStatus.FULL;
        } else {
            status = ChargingStationStatus.AVAILABLE;
        }
    }

    public void occupySlot() {
        if (occupiedSlots < totalSlots) {
            occupiedSlots++;
            updateStatus();
        }
    }

    public void releaseSlot() {
        if (occupiedSlots > 0) {
            occupiedSlots--;
            updateStatus();
        }
    }

    public boolean isAvailable() {
        return status == ChargingStationStatus.AVAILABLE;
    }

    // Getters & setters
    public String getName() { return name; }
    public double getPrice() { return price; }
    public int getTotalSlots() { return totalSlots; }
    public double getPower() { return power; }
    public double getFailureProbability() { return failureProbability; }
    public int getOccupiedSlots() { return occupiedSlots; }
    public ChargingStationStatus getStatus() { return status; }
    public Position getLocation() { return location; }
}
