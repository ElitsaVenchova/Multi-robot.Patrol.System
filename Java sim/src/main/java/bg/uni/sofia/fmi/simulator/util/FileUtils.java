package bg.uni.sofia.fmi.simulator.util;

import java.io.File;

//Used by exporter/logging.
public class FileUtils {

    public static void ensureDirectoryExists(String path) {
        File dir = new File(path);
        if (!dir.exists()) {
            dir.mkdirs();
        }
    }
}