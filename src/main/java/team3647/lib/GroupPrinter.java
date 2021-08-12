package team3647.lib;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GroupPrinter implements Subsystem {
    private final static GroupPrinter INSTANCE = new GroupPrinter();

    public static GroupPrinter getInstance() {
        return INSTANCE;
    }

    ArrayList<DoubleSupplier> toPrint = new ArrayList<>();
    ArrayList<String> keys = new ArrayList<>();

    public synchronized void addDouble(String key, DoubleSupplier function) {
        SmartDashboard.putNumber(key, 0);
        if (key != null && function != null) {
            toPrint.add(function);
            keys.add(key);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < toPrint.size() && i < keys.size(); i++) {
            SmartDashboard.putNumber(keys.get(i), toPrint.get(i).getAsDouble());
        }
    }
}
