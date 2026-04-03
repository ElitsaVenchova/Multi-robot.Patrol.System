package bg.uni.sofia.fmi.simulator.domain;

//[TODO] Да се добави 2D и 3D позиция
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
