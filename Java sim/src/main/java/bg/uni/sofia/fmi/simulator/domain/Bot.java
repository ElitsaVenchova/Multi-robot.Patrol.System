package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.behavior.communication.CommunicationController;
import bg.uni.sofia.fmi.simulator.behavior.communication.CommunicationParticipant;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.behavior.planning.PlanningModule;
import bg.uni.sofia.fmi.simulator.util.GeometryUtils;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;

// Клас, представляващ бот в симулацията
public class Bot {
    private final String name; // име на бота, за по-лесно логване и идентификация
    private final Battery battery; // батерията на бота
    private final Lidar lidar; // сензор за засичане на атаки
    private final CommunicationModule communicationModule; // модула за комуникация
    private final double maxSpeed; // максимална скорост на бота
    private final RobotType type; // тип на бота (наземен, дрон и т.н.)
    private final double failureProbability; // вероятност за повреда при всяко действие
    private final double price; // цена на бота, за оптимизация на разходите
    private final double batteryConsumptionRate; // колко батерия консумира на единица движение
    private final double baselineConsumptionWh; // константна консумация на батерията при всеки tick

    private final long id; // за да може да се идентифицира бота при нужда, напр. за логване
    private final Position position; // позицията на бота
    private final PlanningModule planningModule; // модул за вземане на решения и планиране на действията
    private CommunicationParticipant communicationParticipant;
    private BotState state; // текущо състояние на бота (патрулиране, зареждане, грешка и т.н.)

    private final EnumMap<BotState, Long> stateTicks = new EnumMap<>(BotState.class); // Статистика съхранява броя ticks за всяко състояние на бота

    public Bot(long id, Position position, Battery battery, Lidar lidar, CommunicationModule communicationModule,
               double speed, RobotType type, String name, double failureProbability,
               double price, double batteryConsumptionRate, PlanningModule planningModule, double baselineConsumptionWh) {
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
        this.communicationModule = communicationModule;
        this.planningModule = planningModule;
        this.state = BotState.GOING_TO_PATROL;
        this.baselineConsumptionWh = baselineConsumptionWh;
    }

    // Основен метод за обновяване на състоянието на бота при всяка итерация на симулацията
    public void update(int currentTime) {
        // Взима се решение за действие и се определя състоянието
        planningModule.update(this, currentTime);
        // консумация по подразбиране, за да може да се изтощава с времето дори да не прави нищо
        battery.consume(baselineConsumptionWh);

        // Отчитане на времето прекарано във всеки статус.
        stateTicks.merge(state, 1L, Long::sum);
    }

    // Движение на бота
    public void move(Position target) {
        move(target, GeometryUtils.perimeterDistance(position, target));
    }

    /** Executes a navigation step with its actual path length, including circular arcs. */
    public void move(Position target, double distance) {
        if (!Double.isFinite(distance) || distance < 0 || !Double.isFinite(target.getX())) {
            throw new IllegalArgumentException("Movement must have finite coordinates and non-negative distance");
        }
        double displacement = GeometryUtils.perimeterDistance(position, target);
        double scale = Math.max(Math.max(Math.abs(position.getX()), Math.abs(target.getX())), Math.max(distance, displacement));
        double tolerance = 4 * Math.ulp(Math.max(1.0, scale));
        // Изминатият път не може да е по-кратък от минималното разстояние между позициите.
        if (distance + tolerance < displacement) {
            throw new IllegalArgumentException("Path length cannot be shorter than displacement");
        }
        // Изминатият път не може да надвишава максимално допустимия за една стъпка.
        if(distance > maxSpeed + tolerance){
            throw new RuntimeException("Разстоянието до следващата стъпка е по-голямо от възможностите на робота!");
        }
        position.setX(target.getX());
        battery.consume(distance * batteryConsumptionRate);
    }

    /** Composition wiring; communication delivery does not depend on the patrol interface. */
    public void setCommunicationParticipant(CommunicationParticipant participant) {
        communicationParticipant = participant;
    }

    public void processCommunicationRound(int currentTime, CommunicationController.Round round,
                                          CommunicationController communication) {
        if (communicationParticipant != null) {
            communicationParticipant.onCommunicationRound(this, currentTime, round, communication);
        }
    }

    public Position getPosition() { return position; }
    public Battery getBattery() { return battery; }
    public Lidar getLidar() { return lidar; }
    public CommunicationModule getCommunicationModule() { return communicationModule; }
    public RobotType getType() { return type; }
    public String getName() { return name; }
    public double getFailureProbability() { return failureProbability;}
    public double getPrice() { return price; }
    public double getMaxSpeed() { return maxSpeed; }
    public BotState getState() { return state; }
    public void setState(BotState state) { this.state = state; }
    public PlanningModule getPlanningModule() { return planningModule; }
    public long getId() { return id; }
    public Map<BotState, Long> getStateTicks() { return Collections.unmodifiableMap(stateTicks); }
}
