package bg.uni.sofia.fmi.simulator.domain;

import java.util.ArrayList;
import java.util.List;

public class Perimeter {

    private int size;
    private List<Attack>[] cells;

    public Perimeter(int size) {
        this.size = size;
        this.cells = new ArrayList[size];

        for (int i = 0; i < size; i++) {
            cells[i] = new ArrayList<>();
        }
    }

    public void addAttack(Attack attack) {
        int index = (int) attack.getPosition().getX();

        index = clamp(index);

        cells[index].add(attack);
    }

    public List<Attack> getNearbyAttacks(double x, double range) {

        List<Attack> result = new ArrayList<>();

        int center = (int) x;
        int r = (int) Math.ceil(range);

        int min = clamp(center - r);
        int max = clamp(center + r);

        for (int i = min; i <= max; i++) {
            result.addAll(cells[i]);
        }

        return result;
    }

    private int clamp(int index) {
        if (index < 0)
            return 0;
        if (index >= size)
            return size - 1;
        return index;
    }

    public int getSize() {
        return size;
    }
}
