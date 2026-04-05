package bg.uni.sofia.fmi.simulator.domain;

import java.util.LinkedList;
import java.util.Queue;

import bg.uni.sofia.fmi.simulator.domain.enums.ChargingStationStatus;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;

public class ChargingStation {

    private String name;
    private double price;
    private int totalSlots;
    private double power;
    private double failureProbability;
    private Position location;

    private long id; // за да може да се идентифицира станцията при нужда, напр. за логване
    private ChargingStationStatus status;
    private Queue<Bot> queue = new LinkedList<>();

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

        this.id = IdGenerator.nextId();
        this.status = ChargingStationStatus.AVAILABLE;
    }

    public void chargeBot(Bot bot) {
        bot.getBattery().charge(power);
    }

    public synchronized boolean tryOccupySlot(Bot bot) {

        if (status == ChargingStationStatus.FAIL) {
            return false;
        }

        if (!queue.contains(bot)) {
            queue.add(bot);
        }

        updateStatus();

        // Bot can charge if it is within the first N slots
        return isCharging(bot);
    }

    public synchronized void releaseSlot(Bot bot) {
        queue.remove(bot);
        updateStatus();
    }

    public boolean isCharging(Bot bot) {

        int index = getPositionInQueue(bot);

        return index >= 0 && index < totalSlots;
    }

    private int getPositionInQueue(Bot bot) {

        int i = 0;

        for (Bot b : queue) {
            if (b == bot) {
                return i;
            }
            i++;
        }

        return -1;
    }

    // =========================
    // WAIT TIME ESTIMATION
    // =========================

    public synchronized int getEstimatedWaitTime() {
        int time = 0;

        for (Bot b : queue) {
            time += b.getBattery().estimateTimeToFull(power);
        }
        return time;
    }

    // =========================
    // STATUS
    // =========================

    public void updateStatus() {

        if (status == ChargingStationStatus.FAIL) {
            return;
        }

        if (queue.size() >= totalSlots) {
            status = ChargingStationStatus.FULL;
        } else {
            status = ChargingStationStatus.AVAILABLE;
        }
        System.out.println("ChargingStation " + id + " status: " + status + " queue size: " + queue.size());
    }

    public boolean isAvailable() {
        return status == ChargingStationStatus.AVAILABLE;
    }

    // =========================
    // DERIVED VALUES
    // =========================

    public int getOccupiedSlots() {
        return Math.min(queue.size(), totalSlots);
    }

    public int getQueueSize() {
        return queue.size();
    }

    // =========================
    // GETTERS
    // =========================

    public String getName() {
        return name;
    }

    public double getPrice() {
        return price;
    }

    public int getTotalSlots() {
        return totalSlots;
    }

    public double getPower() {
        return power;
    }

    public double getFailureProbability() {
        return failureProbability;
    }

    public ChargingStationStatus getStatus() {
        return status;
    }

    public Position getLocation() {
        return location;
    }
}
