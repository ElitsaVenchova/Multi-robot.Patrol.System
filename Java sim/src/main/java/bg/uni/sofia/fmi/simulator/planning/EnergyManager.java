package bg.uni.sofia.fmi.simulator.planning;

import bg.uni.sofia.fmi.simulator.domain.Bot;

// Клас за управление на енергията на бота, който може да се използва от поведенческия модул
// [TODO] Може да се разшири с функции за планиране на зареждане, оптимизация на енергията и др.
public class EnergyManager {
    private double threshold; // Праг за ниско ниво на батерията, под който ботът трябва да се зарежда

    public EnergyManager(double threshold) {
        this.threshold = threshold;
    }

    // Метод за проверка дали батерията на бота е под прага
    public boolean isLow(Bot bot) {
        return bot.getBattery().getCurrentLevel() < threshold;
    }
}
