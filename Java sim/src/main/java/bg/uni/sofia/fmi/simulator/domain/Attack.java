package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;

public class Attack {

    private Position position;
    private AttackStatus status;
    private int creationTime;
    private int detectionTime = -1;

    public Attack(Position position, int creationTime) {
        this.position = position;
        this.creationTime = creationTime;
        this.status = AttackStatus.ACTIVE;
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
