package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за периметъра в симулацията
public class PerimeterConfig {
    private int size; // размер на периметъра
    private String type; //името на вида перимтър (линеен, кръг)

    public int getSize() { return size; }

    public void setSize(int size) { this.size = size; }

    public String getType() { return type; }

    public void setType(String type) { this.type = type; }
}