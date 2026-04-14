package bg.uni.sofia.fmi.simulator.config;

import org.yaml.snakeyaml.Yaml;
import java.io.InputStream;

// Клас за зареждане на конфигурацията на моделите на роботите от YAML файлове
public class RobotModelLoader {

    public RobotModelConfig load(String modelName) {

        String path = "configs/robots/" + modelName + ".yaml";

        InputStream inputStream = getClass()
                .getClassLoader()
                .getResourceAsStream(path);

        if (inputStream == null) {
            throw new RuntimeException("Robot model not found: " + modelName);
        }

        Yaml yaml = new Yaml();
        return yaml.loadAs(inputStream, RobotModelConfig.class);
    }
}