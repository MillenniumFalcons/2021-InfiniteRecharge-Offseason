/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.wpi.Timer;

public class StopShooting extends CommandBase {

    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;
    private final Timer timer;

    /**
     * Creates a new StopShooting.
     */
    public StopShooting(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer) {
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;
        timer = new Timer();
        addRequirements(m_flywheel, m_kickerWheel, m_indexer);
        // Use addRequirements() here to declare subsystem dependencies.
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
        m_flywheel.end();
        m_kickerWheel.end();
        m_indexer.end();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > .1;
    }
}
