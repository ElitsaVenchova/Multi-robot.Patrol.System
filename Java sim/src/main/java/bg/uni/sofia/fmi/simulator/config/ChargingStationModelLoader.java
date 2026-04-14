package bg.uni.sofia.fmi.simulator.config;

import org.yaml.snakeyaml.Yaml;

import java.io.InputStream;

// Зарядните станции могат да имат различни модели с различни характеристики, които се дефинират в отделни YAML файлове.
// Този клас зарежда конфигурацията за даден модел на зарядна станция от съответния YAML файл.
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