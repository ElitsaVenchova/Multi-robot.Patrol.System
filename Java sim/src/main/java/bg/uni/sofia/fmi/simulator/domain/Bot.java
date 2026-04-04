package bg.uni.sofia.fmi.simulator.domain;

//[TODO] Да се добави секция, която се охранява
public class Bot {
    private Position position;
    private Battery battery;
    private Lidar lidar;
    private double speed;
    private String type;
    private String name;

    public Bot(Position position, Battery battery, Lidar lidar, double speed, String type, String name) {
        this.position = position;
        this.battery = battery;
        this.lidar = lidar;
        this.speed = speed;
        this.type = type;
        this.name = name;
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

    public Battery getBattery() {
        return battery;
    }

    public Lidar getLidar() {
        return lidar;
    }

    public String getType() {
        return type;
    }

    public String getName() {
        return name;
    }

}
