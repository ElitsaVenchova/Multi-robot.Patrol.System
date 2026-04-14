package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за зарядните станции
public class ChargingStationConfig {
    private String model; //името на модела на зарядната станция
    //позицията може да се задава в конфигурацията, но може и да се [TODO] генерира на базата на броя станции и размера на света
    private double x;
    private double y;

    public String getModel() {
        return model;
    }

    public void setModel(String model) {
        this.model = model;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }
}
