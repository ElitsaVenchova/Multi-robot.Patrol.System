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

    public Position selectStation(Bot bot) {
        // Избираме най-добрата станция за зареждане
        StationSelector selector = new StationSelector();
        ChargingStation best = selector.selectBestStation(bot);
        // Ако няма налична станция
        if (best == null) {
            bot.setState(BotState.WAITING_FOR_CHARGE);
            // Ще потърси отново на следващия ход
            return null;
        }
        bot.setState(BotState.GOING_TO_CHARGE);
        Position goalPosition = best.getLocation();

        double distX = GeometryUtils.perimeterDistance(bot.getPosition(), goalPosition);
        if (distX < 1.0) {
            boolean canCharge = best.tryOccupySlot(bot);
            if (canCharge) {
                currentStation = best;
                bot.setState(BotState.CHARGING);
            } else {
                // чака, [TODO] но ако е заето повече от 5 цикъла, търси друга станция
                bot.setState(BotState.WAITING_FOR_CHARGE);
            }
            return null;
        } else {
            // Насочваме се към станцията
            return goalPosition;
        }
    }

    public void change(Bot bot){
        // Зареждаме бота
        currentStation.chargeBot(bot);
        // Ако батерията е пълна, освобождаваме слота и се връщаме към патрулиране
        if (bot.getBattery().isFull()) {
            currentStation.releaseSlot(bot);
            currentStation = null;
            bot.setState(BotState.PATROLLING);
        }
    }
}
