package bg.uni.sofia.fmi.simulator.config;

public class PatrolConfig {

    private String type;
    private Integer robotsPerSection;

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public Integer getRobotsPerSection() {
        return robotsPerSection;
    }

    public void setRobotsPerSection(Integer robotsPerSection) {
        this.robotsPerSection = robotsPerSection;
    }
}
