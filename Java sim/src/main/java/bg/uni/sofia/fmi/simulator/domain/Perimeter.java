package bg.uni.sofia.fmi.simulator.domain;

import java.util.ArrayList;
import java.util.List;

// Периметърът, който трябва да се патрулира, разделен на клетки, в които могат да се появяват атаки
public class Perimeter {
    private int size; // дължина на периметъра, разделен на клетки
    private List<Attack>[] cells; // масив от списъци, където всяка клетка съдържа атаките, които се намират в нея

    public Perimeter(int size) {
        this.size = size;
        this.cells = new ArrayList[size];

        for (int i = 0; i < size; i++) {
            cells[i] = new ArrayList<>();
        }
    }

    // Добавяне на атака към периметъра, като се поставя в съответната клетка според позицията й
    public void addAttack(Attack attack) {
        int index = (int) attack.getPosition().getX();

        index = clamp(index);

        cells[index].add(attack);
    }

    // Получаване на всички атаки в радиус от дадена позиция
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

    // Помощен метод за ограничаване на индекса в рамките на размера на периметъра
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
