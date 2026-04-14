package bg.uni.sofia.fmi.simulator.util;

import java.io.File;

// За работата с файлове и директории
public class FileUtils {
    // Метод за създаване на директория, ако не съществува
    public static void ensureDirectoryExists(String path) {
        File dir = new File(path);
        if (!dir.exists()) {
            dir.mkdirs();
        }
    }
}