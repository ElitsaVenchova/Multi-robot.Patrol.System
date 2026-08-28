package bg.uni.sofia.fmi.simulator.util;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

// За работа с файлове и директории
public class FileUtils {

    // Метод за създаване на директория, ако не съществува (с поддръжка на String път)
    public static void ensureDirectoryExists(String path) {
        if (path == null || path.trim().isEmpty()) {
            return;
        }
        ensureDirectoryExists(Paths.get(path));
    }

    // Метод за създаване на директория с java.nio.file.Path
    public static void ensureDirectoryExists(Path path) {
        if (path == null) {
            return;
        }
        try {
            if (!Files.exists(path)) {
                Files.createDirectories(path);
            }
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to create directory: " + path, e);
        }
    }

    // Проверка дали файл или директория съществува
    public static boolean exists(String path) {
        return path != null && Files.exists(Paths.get(path));
    }

    // Проверка дали файл или директория съществува
    public static boolean exists(Path path) {
        return path != null && Files.exists(path);
    }

    // Изтриване на файл ако съществува
    public static boolean deleteIfExists(Path path) {
        if (path == null) return false;
        try {
            return Files.deleteIfExists(path);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to delete file: " + path, e);
        }
    }

    // Изтриване на файл по String път ако съществува
    public static boolean deleteIfExists(String path) {
        if (path == null) return false;
        return deleteIfExists(Paths.get(path));
    }
}