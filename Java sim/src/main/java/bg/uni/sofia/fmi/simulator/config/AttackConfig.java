package bg.uni.sofia.fmi.simulator.config;

// attackModel:
//   type: PoissonAttack
//   lambda: 0.3
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
