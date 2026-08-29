package bg.uni.sofia.fmi.simulator.strategy.attack;

import bg.uni.sofia.fmi.simulator.domain.Attack;
import bg.uni.sofia.fmi.simulator.domain.World;

import java.util.List;

/**
 * Модел на групова атака, при който няколко атаки възникват
 * в близък момент и/или в близка пространствена зона.
 * Използва се за изследване на реакцията на патрулната система
 * при концентрирано атакуващо натоварване.
 */
public class GroupAttack implements LoadModel {

    @Override
    public List<Attack> generateAttacks(World world, int currentTime) {
        throw new UnsupportedOperationException("Not implemented yet");
    }
}
