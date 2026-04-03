package bg.uni.sofia.fmi.simulator.config;

// configuration for an attack, e.g. type and parameters such as lambda for exponential distribution of

// [TODO] - duration - missing?
public class AttackConfig {
    private String type;
    private Double lambda; // optional depending on type

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public Double getLambda() {
        return lambda;
    }

    public void setLambda(Double lambda) {
        this.lambda = lambda;
    }
}
