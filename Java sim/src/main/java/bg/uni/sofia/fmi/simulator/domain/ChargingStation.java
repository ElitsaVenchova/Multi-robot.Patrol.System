package bg.uni.sofia.fmi.simulator.domain;

import java.util.LinkedList;
import java.util.Queue;

import bg.uni.sofia.fmi.simulator.domain.enums.ChargingStationStatus;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;

// Зарядна станция, където ботът може да зарежда батерията си
public class ChargingStation {
    private String name; // име на модела на станцията
    private double price; // цена за зареждане, която се използва за оптимизация на разходите
    private int totalSlots; // общ брой слотове за зареждане
    private double power; // мощност на зареждане, която определя колко бързо се зарежда батерията
    private double failureProbability; // вероятност за повреда при всяко действие
    private Position location; // позиция на станцията в света, за да може ботът да се насочва към нея

    private long id; // за да може да се идентифицира станцията при нужда, напр. за логване
    private ChargingStationStatus status; // текущ статус на станцията (напр. свободна, заета, в грешка)
    private Queue<Bot> queue = new LinkedList<>(); // опашка от ботове, които чакат да се заредят

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

    // Зареждане на бота
    public void chargeBot(Bot bot) {
        bot.getBattery().charge(power);
    }

    // Опит за заемане на слот за зареждане. Връща true, ако ботът може
    // да започне да се зарежда веднага, или false, ако трябва да чака.
    public synchronized boolean tryOccupySlot(Bot bot) {
        if (status == ChargingStationStatus.FAIL) {
            return false;
        }

        if (!queue.contains(bot)) {
            queue.add(bot);
        }

        updateStatus();

        // Ако ботът е в първите totalSlots в опашката, може да започне да се зарежда
        return isCharging(bot);
    }

    // Освобождаване на слот след като ботът е заредил.
    public synchronized void releaseSlot(Bot bot) {
        queue.remove(bot);
        updateStatus();
    }

    // Проверка дали ботът в момента се зарежда (т.е. е в първите totalSlots в опашката)
    public boolean isCharging(Bot bot) {
        int index = getPositionInQueue(bot);
        return index >= 0 && index < totalSlots;
    }

    // Получаване на позицията на бота в опашката. Връща -1, ако ботът не е в опашката.
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

    // Оценка за времето на чакане, базирана на броя ботове 
    // в опашката и колко време ще им отнеме да се заредят
    public synchronized int getEstimatedWaitTime() {
        int time = 0;
        for (Bot b : queue) {
            time += b.getBattery().estimateTimeToFull(power);
        }
        return time;
    }

    // Актуализиране на статуса на станцията въз основа на текущата опашка и вероятността за повреда
    public void updateStatus() {
        if (status == ChargingStationStatus.FAIL) {
            return;
        }

        if (queue.size() >= totalSlots) {
            status = ChargingStationStatus.OCCUPIED;
        } else {
            status = ChargingStationStatus.AVAILABLE;
        }
        System.out.println("ChargingStation " + id + " status: " + status + " queue size: " + queue.size());
    }

    // Проверка дали станцията е налична за зареждане (т.е. не е в грешка и има свободни слотове)
    public boolean isAvailable() {
        return status == ChargingStationStatus.AVAILABLE;
    }

    // Размер на опашката
    public int getQueueSize() {
        return queue.size();
    }

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
