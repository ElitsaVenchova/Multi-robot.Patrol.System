package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.behavior.planning.PlanningModule;
import bg.uni.sofia.fmi.simulator.strategy.patrol.PatrolSection;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

// Клас, представляващ бот в симулацията
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
    private final PlanningModule planningModule; // модул за вземане на решения и планиране на действията
    private BotState state; // текущо състояние на бота (патрулиране, зареждане, грешка и т.н.)

    public Bot(long id, Position position, Battery battery, Lidar lidar, double speed, RobotType type, String name,
            double failureProbability, double price, double batteryConsumptionRate, PlanningModule planningModule) {
        this.id = id;
        this.position = position;
        this.battery = battery;
        this.lidar = lidar;
        this.maxSpeed = speed;
        this.type = type;
        this.name = name;
        this.failureProbability = failureProbability;
        this.price = price;
        this.batteryConsumptionRate = batteryConsumptionRate;
        this.planningModule = planningModule;
        this.state = BotState.PATROLLING;
    }

    // Основен метод за обновяване на състоянието на бота при всяка итерация на симулацията
    public void update(int currentTime) {
        // Взима се решение за действие и се определя състоянието
        planningModule.update(this, currentTime);
        // консумация по подразбиране, за да може да се изтощава с времето дори да не прави нищо
        battery.consume(0.01);
    }

    // Движение на бота
    public void move(Position target) {
        double distance = GeometryUtils.perimeterDistance(position, target);
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

    //Проверява дали робота е в секцията за патрулиране
    public boolean isInPatrolSection(PatrolSection patrolSection) {
        if (patrolSection == null) {
            return false;
        }
        if (World.getPerimeter().getType() == PerimeterType.LINEAR) {
            double minX = Math.min(patrolSection.getStartPosition().getX(), patrolSection.getEndPosition().getX());
            double maxX = Math.max(patrolSection.getStartPosition().getX(), patrolSection.getEndPosition().getX());
            return position.getX() >= minX && position.getX() <= maxX;
        } else {
            double x = GeometryUtils.normalizePosition(position.getX());
            double start = GeometryUtils.normalizePosition(patrolSection.getStartPosition().getX());
            double rawEnd = patrolSection.getEndPosition().getX();
            double end = GeometryUtils.normalizePosition(rawEnd);
            // Секцията завършва точно в края на периметъра,  който е физически същата точка като 0.
            boolean endsAtSeam = rawEnd > 0 && end == 0;
            if (endsAtSeam) {
                return x >= start || x < 0.001;
            }
            if (start <= end) { // Обикновена секция: например 20 → 60
                return x >= start && x <= end;
            }
            return x >= start || x <= end; // Секция, която преминава през 0: например 80 → 20
        }
    }

    //Връща по-близкия край на секцията за патрулиране
    public Position getTargetToPatrolSection(PatrolSection patrolSection) {
        if (World.getPerimeter().getType() == PerimeterType.CIRCULAR) {
            Position start = patrolSection.getStartPosition();
            Position end = patrolSection.getEndPosition();
            return GeometryUtils.perimeterDistance(position, start) <= GeometryUtils.perimeterDistance(position, end)
                    ? start : end;
        }
        double minX = Math.min(patrolSection.getStartPosition().getX(), patrolSection.getEndPosition().getX());
        double maxX = Math.max(patrolSection.getStartPosition().getX(), patrolSection.getEndPosition().getX());

        // If bot is outside the section, return the closest boundary position
        double targetX = (position.getX() < minX) ? minX : maxX;
        return new Position(targetX);
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

    public PlanningModule getPlanningModule() {
        return planningModule;
    }

    public long getId() {
        return id;
    }
}
