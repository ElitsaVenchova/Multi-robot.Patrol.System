package bg.uni.sofia.fmi.simulator.domain;

import java.util.ArrayList;
import java.util.List;

import bg.uni.sofia.fmi.simulator.strategy.attack.LoadModel;

// Клас, представляващ света на симулацията, 
// който съдържа всички роботи, атаки, зарядни станции и периметър
public class World {
    private LoadModel attackModel; // Модел за генериране на атаки
    private List<Bot> bots = new ArrayList<>(); // Списък с всички роботи
    private List<Attack> attacks = new ArrayList<>(); // Списък с всички активни атаки
    private List<ChargingStation> chargingStations; // Списък с всички зарядни станции
    private Perimeter perimeter; // Периметърът, който трябва да се патрулира

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

    // Метод, който се извиква при всяка итерация на симулацията, за да се обнови състоянието на света
    // - генериране на нови атаки според модела
    // - обновяване на състоянието на всеки бот (движение, сканиране, зареждане и т.н.)
    // [TODO] patrolModel.execute и bot.update май се препокриват
    public void tick(int currentTime) {
        // Генериране на нови атаки според модела
        List<Attack> newAttacks = attackModel.generateAttacks(this, currentTime);
        for (Attack attack : newAttacks) {
            this.addAttack(attack);
        }

        // Обновяване на състоянието на всеки бот
        for (Bot bot : this.getBots()) {
            bot.update(currentTime);
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

    public void setAttackModel(LoadModel attackModel) {
        this.attackModel = attackModel;
    }
}
