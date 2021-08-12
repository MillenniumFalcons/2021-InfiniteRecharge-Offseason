package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Indexer;
import team3647.lib.IndexerSignal;

public class LoadBalls extends CommandBase {
    private final BallStopper m_ballStopper;
    private final Indexer m_indexer;

    /**
     * Creates a new LoadBalls.
     */
    public LoadBalls(Indexer indexer, BallStopper ballStopper) {
        m_indexer = indexer;
        m_ballStopper = ballStopper;
        addRequirements(m_indexer, m_ballStopper);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_ballStopper.extend();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {        
        if(m_indexer.getBannerSensorValue()) {
            m_indexer.set(IndexerSignal.GO_SLOW);
        } else {
            m_indexer.set(IndexerSignal.GO);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
