package bg.uni.sofia.fmi.simulator.config;

import org.yaml.snakeyaml.Yaml;

import java.io.InputStream;

public class ChargingStationModelLoader {

    public ChargingStationModelConfig load(String modelName) {

        String path = "configs/stations/" + modelName + ".yaml";

        InputStream inputStream = getClass()
                .getClassLoader()
                .getResourceAsStream(path);

        if (inputStream == null) {
            throw new RuntimeException("Station model not found: " + modelName);
        }

        Yaml yaml = new Yaml();
        return yaml.loadAs(inputStream, ChargingStationModelConfig.class);
    }
}