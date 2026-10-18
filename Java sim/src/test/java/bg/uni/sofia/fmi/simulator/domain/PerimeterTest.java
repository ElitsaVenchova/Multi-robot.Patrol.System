package bg.uni.sofia.fmi.simulator.domain;

import bg.uni.sofia.fmi.simulator.domain.enums.AttackStatus;
import bg.uni.sofia.fmi.simulator.domain.enums.PerimeterType;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class PerimeterTest {
    @Test
    public void circularRangeFindsAttacksAcrossSeamWithoutDuplicates() {
        Perimeter perimeter = new Perimeter(10, PerimeterType.CIRCULAR);
        for (int position : new int[]{0, 1, 2, 7, 8, 9}) {
            perimeter.addAttack(new Attack(new Position(position), 0, null));
        }

        List<Attack> nearby = perimeter.getNearbyAttacks(0, 2);

        assertEquals(5, nearby.size());
        assertEquals(5, nearby.stream()
                .map(attack -> (int) attack.getPosition().getX())
                .distinct()
                .count());
        assertTrue(nearby.stream().noneMatch(attack -> attack.getPosition().getX() == 7));
        assertTrue(nearby.stream().anyMatch(attack -> attack.getPosition().getX() == 8));
        assertTrue(nearby.stream().anyMatch(attack -> attack.getPosition().getX() == 9));
    }

    @Test
    public void circularRangeAtLeastHalfThePerimeterReturnsEachCellOnce() {
        Perimeter perimeter = new Perimeter(5, PerimeterType.CIRCULAR);
        for (int position = 0; position < perimeter.getSize(); position++) {
            perimeter.addAttack(new Attack(new Position(position), 0, null));
        }

        List<Attack> nearby = perimeter.getNearbyAttacks(3, 2);

        assertEquals(perimeter.getSize(), nearby.size());
    }

    @Test
    public void circularAttackAtPerimeterLengthIsStoredAtSeam() {
        Perimeter perimeter = new Perimeter(10, PerimeterType.CIRCULAR);
        Attack seamAttack = new Attack(new Position(10), 0, null);

        perimeter.addAttack(seamAttack);

        assertTrue(perimeter.getNearbyAttacks(0, 0).contains(seamAttack));
    }

    @Test
    public void lidarDetectsAttackAcrossCircularSeam() {
        Perimeter perimeter = new Perimeter(10, PerimeterType.CIRCULAR);
        Attack beforeSeam = new Attack(new Position(9), 0, null);
        Attack afterSeam = new Attack(new Position(1), 0, null);
        perimeter.addAttack(beforeSeam);
        perimeter.addAttack(afterSeam);

        new Lidar(1, 0).detect(new Position(0), perimeter, 1);

        assertEquals(AttackStatus.INTERCEPTED, beforeSeam.getStatus());
        assertEquals(AttackStatus.INTERCEPTED, afterSeam.getStatus());
    }

    @Test
    public void linearRangeRemainsClampedAtPerimeterEdges() {
        Perimeter perimeter = new Perimeter(10, PerimeterType.LINEAR);
        Attack atEnd = new Attack(new Position(9), 0, null);
        perimeter.addAttack(atEnd);

        assertTrue(perimeter.getNearbyAttacks(0, 1).isEmpty());
        new Lidar(1, 0).detect(new Position(0), perimeter, 1);
        assertEquals(AttackStatus.ACTIVE, atEnd.getStatus());
    }
}
