package bg.uni.sofia.fmi.simulator.config;

import org.yaml.snakeyaml.Yaml;
import java.io.InputStream;

// Този клас зарежда основната конфигурация за симулацията от YAML файл и 
// валидира данните спрямо предварително дефинирана схема, за да се гарантира, 
// че всички необходими параметри са налични и коректни.
public class ConfigLoader {

    public SimulationConfig load(String path) {
        Yaml yaml = new Yaml();

        //Прочитане на YAML файла с описанието на симулацията от ресурсите на проекта
        InputStream inputStream = getClass()
                .getClassLoader()
                .getResourceAsStream(path);

        if (inputStream == null) {
            throw new RuntimeException("Config file not found: " + path);
        }

        //Десериализация на YAML съдържанието в Java обект от тип SimulationConfig
        SimulationConfig config = yaml.loadAs(inputStream, SimulationConfig.class);
        //Валидиране на заредената конфигурация спрямо предварително дефинираната схема
        SchemaValidator.validate(config);

        return config;
    }
}
