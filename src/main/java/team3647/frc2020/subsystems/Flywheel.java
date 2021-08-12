/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.util.RollingAverage;
import team3647.lib.drivers.SparkMaxFactory;

/**
 * Shoots ball.
 */
public class Flywheel extends SparkMaxSubsystem {

    private final CANSparkMax follower;
    private final RollingAverage velocityAverage;

    public Flywheel(SparkMaxFactory.Configuration masterConfig,
            SparkMaxFactory.Configuration followerConfig, ClosedLoopConfig pidConfig) {
        super(masterConfig, pidConfig);
        super.enableVelocityFiltering();
        follower = addFollower(followerConfig, true);
        velocityAverage = new RollingAverage(10);
        setToBrake();
    }

    public void setRPM(double RPM) {
        setVelocity(RPM);
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        velocityAverage.add(getVelocity());
    }

    public String getName() {
        return "Flywheel";
    }

    @Override
    public void setToBrake() {
        super.setToBrake();
        follower.setIdleMode(IdleMode.kBrake);
    }
}
