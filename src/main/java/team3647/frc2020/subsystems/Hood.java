/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Add your docs here.
 */
public class Hood implements PeriodicSubsystem {

    private class PeriodicIO {
        public double demand;
    }

    private final Servo linearActuator;
    private final double minPosition;
    private final double maxPosition;
    private PeriodicIO periodicIO = new PeriodicIO();


    public Hood(int linearActuatorPWM, double minPosition, double maxPosition) {
        linearActuator = new Servo(linearActuatorPWM);
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        periodicIO.demand = 0;
    }

    @Override
    public void writePeriodicOutputs() {
        linearActuator.set(periodicIO.demand);
    }

    public void setPosition(double pos) {
        periodicIO.demand = MathUtil.clamp(Math.abs(pos), minPosition, maxPosition);
    }

    public double getAppliedPosition() {
        return periodicIO.demand;
    }

    @Override
    public String getName() {
        return "Hood";
    }
}
