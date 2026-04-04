package bg.uni.sofia.fmi.simulator.config;

// configuration for an robot, e.g. model and count

//Example configuration for robots in the simulation, e.g. model and count
// robots:
//   - model: turtlebot_pi
//     count: 5
public class RobotConfig {
    private String model;
    private int count;
    private String type;

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

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }
}