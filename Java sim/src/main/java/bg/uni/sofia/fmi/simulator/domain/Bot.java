package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
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
    }

    // [TODO] Да има посока и цел, но все пак да има леко произвилно джижение
    // Произволността поможе би да е конфигурация
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

}
