package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;

public class EnergyManager {

    private double threshold;

    public EnergyManager(double threshold) {
        this.threshold = threshold;
    }

    public boolean isLow(Bot bot) {
        return bot.getBattery().getCurrentLevel() < threshold;
    }
}
