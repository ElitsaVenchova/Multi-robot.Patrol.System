package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за роботите в симулацията
public class RobotConfig {
    private String model; //името на вида робот
    private int count; // брой роботи от този вид
    
    public String getModel() {
        return model;
    }

    public void setModel(String model) {
        this.model = model;
    }

    public int getCount() {
        return count;
    }

    public void setCount(int count) {
        this.count = count;
    }
}