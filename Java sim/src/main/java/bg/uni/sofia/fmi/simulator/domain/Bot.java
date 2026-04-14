package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.planning.BehaviorModule;
import bg.uni.sofia.fmi.simulator.planning.EnergyManager;
import bg.uni.sofia.fmi.simulator.planning.Navigation;
import bg.uni.sofia.fmi.simulator.planning.ObstacleAvoidance;
import bg.uni.sofia.fmi.simulator.planning.SimpleBehaviorController;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

// Клас, представляващ бот в симулацията
//[TODO] Да се добави секция, която се охранява
public class Bot {
    private String name; // име на бота, за по-лесно логване и идентификация
    private Battery battery; // батерията на бота
    private Lidar lidar; // сензор за засичане на атаки
    private double maxSpeed; // максимална скорост на бота
    private RobotType type; // тип на бота (наземен, дрон и т.н.)
    private double failureProbability; // вероятност за повреда при всяко действие
    private double price; // цена на бота, за оптимизация на разходите
    private double batteryConsumptionRate; // колко батерия консумира на единица движение

    private long id; // за да може да се идентифицира бота при нужда, напр. за логване
    private Position position; // позицията на бота
    private BehaviorModule behavior; // модул за вземане на решения и планиране на действията
    private BotState state; // текущо състояние на бота (патрулиране, зареждане, грешка и т.н.)
    private ChargingStation currentStation; // за да знаем на коя станция се зареждаме
    private ChargingStation targetStation; // за да знаем към коя станция се насочваме

    public Bot(Position position, Battery battery, Lidar lidar, double speed, RobotType type, String name,
            double failureProbability, double price, double batteryConsumptionRate, EnergyManager energyManager) {
        this.position = position;
        this.battery = battery;
        this.lidar = lidar;
        this.maxSpeed = speed;
        this.type = type;
        this.name = name;
        this.failureProbability = failureProbability;
        this.price = price;
        this.batteryConsumptionRate = batteryConsumptionRate;

        this.id = IdGenerator.nextId();
        this.behavior = new SimpleBehaviorController(energyManager, new Navigation(new ObstacleAvoidance()));
        this.state = BotState.PATROLLING;
    }

    // Основен метод за обновяване на състоянието на бота при всяка итерация на симулацията
    public void update(World world, int currentTime) {
        // Взима се решение за действие и се определя състоянието
        behavior.update(this, world, currentTime);

        // Сканиране с лидара за нарушители
        if (state != BotState.ERROR && state != BotState.CHARGING) {
            lidar.detect(position, world.getPerimeter(), currentTime);
            battery.consume(this.lidar.getBatteryConsumptionRate());
        }

        // консумация по подразбиране, за да може да се изтощава с времето дори да не прави нищо
        battery.consume(0.01); 

        System.out.println("Bot " + id + " at " + position + " state: " + state);
    }

    // Движение на бота
    // [TODO] Да има леко произвилно джижение. Произволността може би да е конфигурация
    // [TODO] Някак това движение е безцелно. По-скоро трябва да има параметър с посоки
    public void move() {
        if (!battery.isEmpty()) {
            // always move along perimeter (x)
            position.setX(position.getX() + maxSpeed);

            if (type == RobotType.GROUND) {
                // 2D only
                position.setZ(0.0);

                // optional: small Y variation
                position.setY(position.getY());

            } else if (type == RobotType.DRONE) {
                // 3D movement
                position.setY(position.getY() + randomOffset());
                position.setZ(position.getZ() + randomOffset());
            }
            battery.consume(this.maxSpeed * this.batteryConsumptionRate);
        }
    }


    private double randomOffset() {
        return (RandomProvider.nextDouble() - 0.5) * 0.1;
    }

    public Position getPosition() {
        return position;
    }

    public Battery getBattery() {
        return battery;
    }

    public Lidar getLidar() {
        return lidar;
    }

    public RobotType getType() {
        return type;
    }

    public String getName() {
        return name;
    }

    public double getFailureProbability() {
        return failureProbability;
    }

    public double getPrice() {
        return price;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public BotState getState() {
        return state;
    }

    public void setState(BotState state) {
        this.state = state;
    }

    public void setBehavior(BehaviorModule behavior) {
        this.behavior = behavior;
    }

    public ChargingStation getCurrentStation() {
        return currentStation;
    }

    public void setCurrentStation(ChargingStation chargintStation) {
        this.currentStation = chargintStation;
    }

    public ChargingStation getTargetStation() {
        return targetStation;
    }

    public void setTargetStation(ChargingStation targetStation) {
        this.targetStation = targetStation;
    }
}
