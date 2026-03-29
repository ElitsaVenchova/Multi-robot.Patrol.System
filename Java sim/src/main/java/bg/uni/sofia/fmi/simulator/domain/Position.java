package bg.uni.sofia.fmi.simulator.domain;

public class Position {
    private double value;

    public Position(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

    public void move(double distance) {
        this.value += distance;
    }
}
