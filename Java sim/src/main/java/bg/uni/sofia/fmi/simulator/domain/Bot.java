package bg.uni.sofia.fmi.simulator.domain;

public class Bot {
private Position position;
    private Battery battery;
    private Lidar lidar;
    private double speed;

    public Bot(Position position, Battery battery, Lidar lidar, double speed) {
        this.position = position;
        this.battery = battery;
        this.lidar = lidar;
        this.speed = speed;
    }

    public void move() {
        if (!battery.isEmpty()) {
            position.move(speed);
            battery.consume();
        }
    }

    public boolean detect(Attack attack) {
        return lidar.detect(position, attack.getPosition());
    }

    public Position getPosition() {
        return position;
    }
}
