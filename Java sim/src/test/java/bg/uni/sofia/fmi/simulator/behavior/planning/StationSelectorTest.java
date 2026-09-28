package bg.uni.sofia.fmi.simulator.behavior.planning;

import bg.uni.sofia.fmi.simulator.domain.Battery;
import bg.uni.sofia.fmi.simulator.domain.Bot;
import bg.uni.sofia.fmi.simulator.domain.ChargingStation;
import bg.uni.sofia.fmi.simulator.domain.Lidar;
import bg.uni.sofia.fmi.simulator.domain.Position;
import bg.uni.sofia.fmi.simulator.domain.enums.RobotType;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertSame;

public class StationSelectorTest {

    @Test
    public void selectsNearestStationAcrossCircularSeam() {
        Bot bot = new Bot(1, new Position(990), new Battery(100), new Lidar(0, 0),
                1, RobotType.GROUND, "bot", 0, 0, 0, null);
        ChargingStation acrossSeam = stationAt(10);
        ChargingStation onDirectPath = stationAt(500);

        ChargingStation selected = new StationSelector().selectBestStation(
                bot, List.of(acrossSeam, onDirectPath));

        assertSame(acrossSeam, selected);
    }

    private ChargingStation stationAt(double x) {
        return new ChargingStation("station", 0, 1, 1, 0, new Position(x));
    }
}
