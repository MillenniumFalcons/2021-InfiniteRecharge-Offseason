/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Turret;
import team3647.lib.wpi.Timer;

public class TurretMotionMagic extends CommandBase {
    private final Turret m_turret;
    private final double kAngle;
    private final Timer timer;
    private boolean keepPosition = false;

    /**
     * Creates a new TurretGoTo.
     */
    //angle in deg
    public TurretMotionMagic(Turret turret, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_turret = turret;
        kAngle = angle;
        timer = new Timer();
        keepPosition = false;
        addRequirements(m_turret);
    }


    public TurretMotionMagic keepPosition() {
        keepPosition = true;
        return this;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_turret.setAngle(kAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!keepPosition) {
            m_turret.end();
        } else {
            m_turret.setAngle(kAngle);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_turret.reachedTargetPosition() || timer.get() > .5);
    }
}
