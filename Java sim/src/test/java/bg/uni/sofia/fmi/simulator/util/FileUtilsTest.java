package bg.uni.sofia.fmi.simulator.util;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class FileUtilsTest {

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    @Test
    public void testEnsureDirectoryExists() throws IOException {
        File baseDir = tempFolder.newFolder("sim_test");
        Path subDir = baseDir.toPath().resolve("nested/dir/structure");

        assertFalse(FileUtils.exists(subDir));
        FileUtils.ensureDirectoryExists(subDir);
        assertTrue(FileUtils.exists(subDir));

        // Calling again should not fail
        FileUtils.ensureDirectoryExists(subDir);
        assertTrue(FileUtils.exists(subDir.toString()));
    }

    @Test
    public void testDeleteIfExists() throws IOException {
        File file = tempFolder.newFile("test_file.txt");
        assertTrue(FileUtils.exists(file.toPath()));

        assertTrue(FileUtils.deleteIfExists(file.toPath()));
        assertFalse(FileUtils.exists(file.toPath()));
        assertFalse(FileUtils.deleteIfExists(file.toPath()));
    }
}
