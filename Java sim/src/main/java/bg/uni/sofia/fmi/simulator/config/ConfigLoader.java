package bg.uni.sofia.fmi.simulator.config;

import org.yaml.snakeyaml.Yaml;
import java.beans.Introspector;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

/** Loads scenarios and self-contained experiment snapshots from disk or classpath. */
public class ConfigLoader {
    public SimulationConfig load(String path) {
        SimulationConfig config = read(path, SimulationConfig.class);
        SchemaValidator.validate(config);
        return config;
    }
    public static <T> T read(String path, Class<T> type) {
        try (InputStream stream = open(path)) {
            return new Yaml().loadAs(new InputStreamReader(stream, StandardCharsets.UTF_8), type);
        } catch (IOException e) { throw new IllegalArgumentException("Cannot read configuration: " + path, e); }
    }
    public static InputStream open(String path) throws IOException {
        if (Files.isRegularFile(Path.of(path))) return Files.newInputStream(Path.of(path));
        InputStream stream = ConfigLoader.class.getClassLoader().getResourceAsStream(path);
        if (stream == null) throw new FileNotFoundException(path);
        return stream;
    }
    /** Plain YAML without Java object tags. */
    public static String toYaml(Object value) { return new Yaml().dump(tree(value)); }
    public static SimulationConfig copy(SimulationConfig value) {
        return new Yaml().loadAs(toYaml(value), SimulationConfig.class);
    }
    private static Object tree(Object value) {
        if (value == null || value instanceof String || value instanceof Number || value instanceof Boolean) return value;
        if (value instanceof Enum<?>) return ((Enum<?>) value).name();
        if (value instanceof Map<?, ?>) {
            Map<String, Object> result = new TreeMap<>();
            ((Map<?, ?>) value).forEach((k, v) -> result.put(k.toString(), tree(v)));
            return result;
        }
        if (value instanceof Iterable<?>) {
            List<Object> result = new ArrayList<>();
            for (Object item : (Iterable<?>) value) result.add(tree(item));
            return result;
        }
        try {
            Map<String, Object> result = new TreeMap<>();
            for (var property : Introspector.getBeanInfo(value.getClass(), Object.class).getPropertyDescriptors()) {
                if (property.getReadMethod() != null && property.getWriteMethod() != null)
                    result.put(property.getName(), tree(property.getReadMethod().invoke(value)));
            }
            return result;
        } catch (Exception e) { throw new IllegalArgumentException("Cannot serialize configuration", e); }
    }
}
