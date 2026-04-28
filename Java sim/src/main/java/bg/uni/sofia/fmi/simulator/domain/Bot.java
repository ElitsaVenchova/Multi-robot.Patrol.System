package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.planning.BehaviorModule;
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
    private double direction = 1.0; // +1 or -1 за посока на движение по периметъра
    private World world; // референция към света, в който се намира бота. В бъдеще ще е света, в който си мисли, че се намира спрямо събраната информация от сензорит/комуникация

    public Bot(Position position, Battery battery, Lidar lidar, double speed, RobotType type, String name,
            double failureProbability, double price, double batteryConsumptionRate, BehaviorModule behavior, World world) {
        this.position = position;
        this.battery = battery;
        this.lidar = lidar;
        this.maxSpeed = speed;
        this.type = type;
        this.name = name;
        this.failureProbability = failureProbability;
        this.price = price;
        this.batteryConsumptionRate = batteryConsumptionRate;
        this.world = world;

        //[TODO] Инициализация на поведението, ако е необходимо. Може би някои стратегии имат нужда от референция към света или бота, за да се инициализират правилно
        // patrolModel.initialize(this);

        this.id = IdGenerator.nextId();
        this.behavior = behavior;
        this.state = BotState.PATROLLING;
    }

    // Основен метод за обновяване на състоянието на бота при всяка итерация на симулацията
    public void update(int currentTime) {
        // Взима се решение за действие и се определя състоянието
        behavior.update(this, currentTime);

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
    // [TODO] Размерът на периметъра да се смени с размера на охраняемата секция
    public void move() {
        if (battery.isEmpty()) return;

        double newX = position.getX() + direction * maxSpeed;

        // 🔥 Clamp to perimeter
        double max = world.getPerimeter().getSize();

        if (newX > max) {
            newX = max;
            direction = -1; // bounce back
        } else if (newX < 0) {
            newX = 0;
            direction = 1;
        }

        position.setX(newX);

        // keep other axes simple
        if (type == RobotType.GROUND) {
            position.setZ(0.0);
        } else if (type == RobotType.DRONE) {
            position.setY(position.getY() + randomOffset());
            position.setZ(position.getZ() + randomOffset());
        }

        battery.consume(maxSpeed * batteryConsumptionRate);
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

    public BehaviorModule getBehavior() {
        return behavior;
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

    public void setDirection(double direction) {
        this.direction = direction;
    }

    public long getId() {
        return id;
    }

    public World getWorld() {
        return world;
    }
}
