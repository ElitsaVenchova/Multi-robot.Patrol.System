package bg.uni.sofia.fmi.simulator.domain;

public class Lidar {
    private double range;

    public Lidar(double range) {
        this.range = range;
    }

    public boolean detect(Position botPosition, Position attackPosition) {
        return Math.abs(botPosition.getValue() - attackPosition.getValue()) <= range;
    }
}
