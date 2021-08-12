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
public class Climber implements PeriodicSubsystem {
    private final Solenoid climberRelease;

    public Climber(int solenoidPin) {
        climberRelease = new Solenoid(solenoidPin);
    }

    public void release() {
        climberRelease.set(true);
    }

    public void lock() {
        climberRelease.set(false);
    }

    @Override
    public String getName() {
        return "Climber";
    }
}
