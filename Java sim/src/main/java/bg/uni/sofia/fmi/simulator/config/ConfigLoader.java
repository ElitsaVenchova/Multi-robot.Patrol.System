package bg.uni.sofia.fmi.simulator.config;

import org.yaml.snakeyaml.Yaml;
import java.io.InputStream;

//This is DSL interpreter entry point.
public class ConfigLoader {

    public SimulationConfig load(String path) {
        Yaml yaml = new Yaml();

        InputStream inputStream = getClass()
                .getClassLoader()
                .getResourceAsStream(path);

        if (inputStream == null) {
            throw new RuntimeException("Config file not found: " + path);
        }

        SimulationConfig config = yaml.loadAs(inputStream, SimulationConfig.class);

        SchemaValidator.validate(config);

        return config;
    }
}
