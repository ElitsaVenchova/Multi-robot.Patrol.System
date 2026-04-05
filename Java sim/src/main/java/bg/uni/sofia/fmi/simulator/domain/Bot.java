package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.BotState;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import bg.uni.sofia.fmi.simulator.planning.BehaviorModule;
import bg.uni.sofia.fmi.simulator.util.RandomProvider;

//[TODO] Да се добави секция, която се охранява
public class Bot {
    private Battery battery;
    private Lidar lidar;
    private double maxSpeed;
    private RobotType type;
    private String name;
    private double failureProbability;
    private double price;
    private double batteryConsumptionRate;

    private Position position;
    private BehaviorModule behavior;
    private BotState state;
    private ChargingStation currentStation; // за да знаем на коя станция се зареждаме
    private ChargingStation targetStation; // за да знаем към коя станция се насочваме

    public Bot(Position position, Battery battery, Lidar lidar, double speed, RobotType type, String name,
            double failureProbability, double price, double batteryConsumptionRate) {
        this.position = position;
        this.battery = battery;
        this.lidar = lidar;
        this.maxSpeed = speed;
        this.type = type;
        this.name = name;
        this.failureProbability = failureProbability;
        this.price = price;
        this.batteryConsumptionRate = batteryConsumptionRate;

        this.state = BotState.PATROLLING;
    }

    public void update(World world, int currentTime) {
        System.out.println("Bot " + position + " state: " + state);
        // 1. Decision making (planning)
        behavior.update(this, world, currentTime);

        // 2. Detection only if active
        if (state != BotState.ERROR && state != BotState.CHARGING) {
            lidar.detect(position, world.getPerimeter(), currentTime);
        }

        // 3. Energy handling
        // [TODO] Това трябва да е по-сложно. Трябва да се отчита какво е правил.
        // Най-добре е да се сложи по едно извикване за всяко дейсвие + консумация по подразбиране, напр. 0.01.
        if (state != BotState.CHARGING) {
            battery.consume(this.maxSpeed * this.batteryConsumptionRate);
            battery.consume(this.lidar.getBatteryConsumptionRate());
        }

        if (state == BotState.CHARGING) {
            battery.charge(currentStation.getPower()); // you implement simple version
        }
    }

    // [TODO] Да има леко произвилно джижение. Произволността може би да е
    // конфигурация
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
