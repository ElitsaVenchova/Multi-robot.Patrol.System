package bg.uni.sofia.fmi.simulator.behavior.energyManagment;

import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

// Клас за управление на енергията на бота, който може да се използва от поведенческия модул
// [TODO] Може да се разшири с функции за планиране на зареждане, оптимизация на енергията и др.
public class EnergyManager {
    private final double threshold; // Праг за ниско ниво на батерията, под който ботът трябва да се зарежда

    private ChargingStation currentStation; // за да знаем на коя станция се зареждаме

    public EnergyManager(double threshold) {
        this.threshold = threshold;
    }

    // Метод за проверка дали батерията на бота е под прага
    public boolean isLow(Bot bot) {
        return bot.getBattery().getCurrentLevel() < threshold;
    }

    public ChargingStation selectStation(Bot bot) {
        // Избираме най-добрата станция за зареждане
        StationSelector selector = new StationSelector();
        ChargingStation best = selector.selectBestStation(bot);
        // Ако няма налична станция
        if (best == null) { // Ще потърси отново на следващия ход
            return null;
        }
        return best;
    }

    public boolean tryToCharge(ChargingStation best, Bot bot){
        boolean canCharge = best.tryOccupySlot(bot);
        if (canCharge) {
            currentStation = best;
        }
        return canCharge;
    }

    //Зарежда робота и връща дали батерията е пълна
    public boolean change(Bot bot){
        // Зареждаме бота
        currentStation.chargeBot(bot);
        // Ако батерията е пълна, освобождаваме слота и се връщаме към патрулиране
        if (bot.getBattery().isFull()) {
            currentStation.releaseSlot(bot);
            currentStation = null;
        }
        return bot.getBattery().isFull();
    }
}
