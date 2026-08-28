package bg.uni.sofia.fmi.simulator.util;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class IdGeneratorTest {

    @Before
    public void setUp() {
        IdGenerator.reset();
    }

    @Test
    public void testSequentialIdsAndReset() {
        assertEquals(0L, IdGenerator.get());
        assertEquals(1L, IdGenerator.nextId());
        assertEquals(2L, IdGenerator.nextId());
        assertEquals(3L, IdGenerator.nextId());
        assertEquals(3L, IdGenerator.get());

        IdGenerator.reset();
        assertEquals(0L, IdGenerator.get());
        assertEquals(1L, IdGenerator.nextId());
    }
}
