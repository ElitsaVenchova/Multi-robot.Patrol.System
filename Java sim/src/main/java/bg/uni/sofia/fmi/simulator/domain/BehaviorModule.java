package bg.uni.sofia.fmi.simulator.domain;

public interface BehaviorModule {

    void update(Bot bot, World world, int currentTime);

}