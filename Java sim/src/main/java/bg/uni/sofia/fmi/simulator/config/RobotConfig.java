package bg.uni.sofia.fmi.simulator.config;

// configuration for an robot, e.g. model and count

//Example configuration for robots in the simulation, e.g. model and count
// robots:
//   - model: turtlebot_pi
//     count: 5

//  - speed - missing?
//  - sensingRange - missing?
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