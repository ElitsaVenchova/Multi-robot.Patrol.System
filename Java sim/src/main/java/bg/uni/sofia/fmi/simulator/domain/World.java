package bg.uni.sofia.fmi.simulator.domain;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;

// Клас, представляващ света на симулацията, 
// който съдържа всички роботи, атаки, зарядни станции и периметър
public class World {
    private LoadModel attackModel; // Модел за генериране на атаки

    private static final List<Bot> bots = new ArrayList<>(); // Списък с всички роботи
    private static List<ChargingStation> chargingStations; // Списък с всички зарядни станции
    private static Perimeter perimeter = null; // Периметърът, който трябва да се патрулира

    public void addBots(List<Bot> bots) {
        World.bots.addAll(bots);
    }

    public void addChargingStations(List<ChargingStation> chargingStations) {
        World.chargingStations = chargingStations;
    }

    // Метод, който се извиква при всяка итерация на симулацията, за да се обнови състоянието на света
    // - генериране на нови атаки според модела
    // - обновяване на състоянието на всеки бот (движение, сканиране, зареждане и т.н.)
    public void tick(int currentTime) {
        // Генериране на нови атаки според модела
        List<Attack> newAttacks = attackModel.generateAttacks(currentTime);
        for (Attack attack : newAttacks) {
            if(perimeter != null){
                perimeter.addAttack(attack);
            }
        }

        // Обновяване на състоянието на всеки бот
        for (Bot bot : World.getBots()) {
            bot.update(currentTime);
        }
    }

    public static Perimeter getPerimeter() {
        return perimeter;
    }

    public static List<Bot> getBots() {
        return bots;
    }

    public static List<ChargingStation> getChargingStations() {
        return chargingStations;
    }

    public void setAttackModel(LoadModel attackModel) {
        this.attackModel = attackModel;
    }

    public static void setPerimeter(Perimeter perimeter) { World.perimeter = perimeter;}
}
