package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.planning.BehaviorModule;
import bg.uni.sofia.fmi.simulator.util.IdGenerator;

// Клас, представляващ бот в симулацията
//[TODO] Да се добави секция, която се охранява
public class Bot {
    private final String name; // име на бота, за по-лесно логване и идентификация
    private final Battery battery; // батерията на бота
    private final Lidar lidar; // сензор за засичане на атаки
    private final double maxSpeed; // максимална скорост на бота
    private final RobotType type; // тип на бота (наземен, дрон и т.н.)
    private final double failureProbability; // вероятност за повреда при всяко действие
    private final double price; // цена на бота, за оптимизация на разходите
    private final double batteryConsumptionRate; // колко батерия консумира на единица движение

    private final long id; // за да може да се идентифицира бота при нужда, напр. за логване
    private final Position position; // позицията на бота
    private BehaviorModule behavior; // модул за вземане на решения и планиране на действията
    private BotState state; // текущо състояние на бота (патрулиране, зареждане, грешка и т.н.)

    public Bot(Position position, Battery battery, Lidar lidar, double speed, RobotType type, String name,
            double failureProbability, double price, double batteryConsumptionRate, BehaviorModule behavior) {
        this.position = position;
        this.battery = battery;
        this.lidar = lidar;
        this.maxSpeed = speed;
        this.type = type;
        this.name = name;
        this.failureProbability = failureProbability;
        this.price = price;
        this.batteryConsumptionRate = batteryConsumptionRate;

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
//        if (state != BotState.ERROR && state != BotState.CHARGING) {
//            lidar.detect(position, world.getPerimeter(), currentTime);
//            battery.consume(this.lidar.getBatteryConsumptionRate());
//        }

        // консумация по подразбиране, за да може да се изтощава с времето дори да не прави нищо
        battery.consume(0.01); 

        System.out.println("Bot " + id + " at " + position + " state: " + state);
    }

    // Движение на бота
    public void move(Position target) {
        double dx = target.getX() - position.getX();
        double distance = Math.abs(dx);
        if(distance > maxSpeed){
            throw new RuntimeException("Разстоянието до следващата стъпка е по-голоямо от възможностите на робота!");
        }

        position.setX(target.getX());
        // Засега 1D движение
        if (type == RobotType.GROUND) {
            position.setZ(0.0);
        } else if (type == RobotType.DRONE) {
            position.setY(100);
            position.setZ(0.0);
        }

        battery.consume(distance * batteryConsumptionRate);
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

    public long getId() {
        return id;
    }
}
