package bg.uni.sofia.fmi.simulator.domain;

import java.util.ArrayList;
import java.util.List;

public class World {

    private List<Bot> bots = new ArrayList<>();
    private List<Attack> attacks = new ArrayList<>();
    private List<ChargingStation> chargingStations;
    private Perimeter perimeter;

    public World(double perimeter) {
        this.perimeter = new Perimeter((int) perimeter);
    }

    public void addBots(List<Bot> bots) {
        this.bots.addAll(bots);
    }

    public void addAttack(Attack attack) {
        attacks.add(attack);
    }

    public void addChargingStations(List<ChargingStation> chargingStations) {
        this.chargingStations = chargingStations;
    }

    public void tick(int currentTime) {
        // Move robots
        for (Bot bot : bots) {
            bot.update(this, currentTime);
        }
    }

    public List<Attack> getAttacks() {
        return attacks;
    }

    public Perimeter getPerimeter() {
        return perimeter;
    }

    public List<Bot> getBots() {
        return bots;
    }

    public List<ChargingStation> getChargingStations() {
        return chargingStations;
    }
}
