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
import team3647.frc2020.subsystems.Turret;

public class AimTurret extends CommandBase {

    private final Turret m_turret;
    private final DoubleSupplier m_angleToTarget;
    private final BooleanSupplier hasValidTarget;
    private double turretPositionToGoTo = 0;

    /**
     * Creates a new AimTurret.
     */
    public AimTurret(Turret turret, DoubleSupplier angleToTurret, BooleanSupplier hasValidTarget) {
        m_turret = turret;
        m_angleToTarget = angleToTurret;
        this.hasValidTarget = hasValidTarget;
        addRequirements(m_turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_turret.setAiming(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(hasValidTarget.getAsBoolean()) {
            turretPositionToGoTo = m_turret.getAngle() - m_angleToTarget.getAsDouble();
        } else {
            turretPositionToGoTo = 0;
        }
        m_turret.setAngle(turretPositionToGoTo);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.end();
        m_turret.setAiming(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
