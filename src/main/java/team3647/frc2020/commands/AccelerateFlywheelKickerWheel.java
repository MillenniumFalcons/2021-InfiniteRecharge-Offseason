/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.KickerWheel;

public class AccelerateFlywheelKickerWheel extends CommandBase {
    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final DoubleSupplier flywheelRPM;
    private final BooleanSupplier hasValidTarget;

    /**
     * Creates a new AccelerateFlywheel.
     */
    public AccelerateFlywheelKickerWheel(Flywheel flywheel, KickerWheel kickerWheel, DoubleSupplier flywheelRPM,
            BooleanSupplier hasValidTarget) {
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        this.flywheelRPM = flywheelRPM;
        this.hasValidTarget = hasValidTarget;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_flywheel, m_kickerWheel);
    }

    /**
     * Creates a new AccelerateFlywheel.
     */
    public AccelerateFlywheelKickerWheel(Flywheel flywheel, KickerWheel kickerWheel, double flywheelRPM,
            boolean hasValidTarget) {
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        this.flywheelRPM = () -> {
            return flywheelRPM;
        };
        this.hasValidTarget = () -> {
            return hasValidTarget;
        };
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_flywheel, m_kickerWheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (hasValidTarget.getAsBoolean()) {
            m_flywheel.setRPM(flywheelRPM.getAsDouble());
            m_kickerWheel.setOpenloop(.5);
            System.out.println("Running shooter");
        } else {
            System.out.println("Stopping shooter");
            m_flywheel.setOpenloop(0);
            m_kickerWheel.setOpenloop(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return m_flywheel.reachedTargetVelocity();
        return false;
    }
}
