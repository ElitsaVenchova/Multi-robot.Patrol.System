package bg.uni.sofia.fmi.simulator.domain;

public class SimpleBehaviorController implements BehaviorModule {

    private EnergyManager energyManager;
    private Navigation navigation;

    public SimpleBehaviorController(EnergyManager energyManager,
                                    Navigation navigation) {
        this.energyManager = energyManager;
        this.navigation = navigation;
    }

    @Override
    public void update(Bot bot, World world, int currentTime) {

        if (energyManager.isLow(bot)) {
            navigation.goToChargingStation(bot, world);
        } else {
            navigation.patrol(bot, world);
        }
    }
}
