/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import team3647.lib.wpi.Solenoid;

/**
 * Add your docs here.
 */
public class BallStopper implements PeriodicSubsystem {
    private final Solenoid stopPistons;

    public BallStopper(int solenoidPin) {
        stopPistons = new Solenoid(solenoidPin);
    }

    public void extend() {
        set(true);
    }

    public void retract() {
        set(false);
    }

    public void set(boolean value) {
        stopPistons.set(value);
    }

    @Override
    public String getName() {
        return "BallStopper";
    }
}
