package bg.uni.sofia.fmi.simulator.config;

// robots:
//   - model: turtlebot_pi
//     count: 5
    // public double speed;
    // public double sensingRange;
    // public List<Double> initialPositions;
public class RobotConfig {

    private String model;
    private int count;

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