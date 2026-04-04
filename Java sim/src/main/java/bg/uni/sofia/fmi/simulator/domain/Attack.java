package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

public class Attack {

    private Position position;
    private AttackStatus status;
    private int creationTime;
    private int detectionTime = -1;
    private Integer duration; // null = infinite

    public Attack(Position position, int creationTime, Integer duration) {
        this.position = position;
        this.creationTime = creationTime;
        this.duration = duration;
        this.status = AttackStatus.ACTIVE;
    }

    public boolean isExpired(int currentTime) {
        if (duration == null)
            return false;

        return (currentTime - creationTime) > duration;
    }

    public void intercept(int time) {
        this.status = AttackStatus.INTERCEPTED;
        this.detectionTime = time;
    }

    public Position getPosition() {
        return position;
    }

    public AttackStatus getStatus() {
        return status;
    }

    public void intercept() {
        this.status = AttackStatus.INTERCEPTED;
    }

    public void miss() {
        this.status = AttackStatus.MISSED;
    }

    public int getCreationTime() {
        return creationTime;
    }

    public int getDetectionTime() {
        return detectionTime;
    }
}
