package bg.uni.sofia.fmi.simulator.config;

// Конфигурация за атаките
public class AttackConfig {
    private String model; //името на вида атака
    private Double lambda; // опционално за атаки с експоненциално разпределение
    private Integer duration; // опционално за атаки с фиксирана продължителност. Ако не е зададено, атаката продължава до засичането й.

    public String getModel() {
        return model;
    }

    public void setModel(String type) {
        this.model = type;
    }

    public Double getLambda() {
        return lambda;
    }

    public void setLambda(Double lambda) {
        this.lambda = lambda;
    }

    public Integer getDuration() {
        return duration;
    }

    public void setDuration(Integer duration) {
        this.duration = duration;
    }
}
