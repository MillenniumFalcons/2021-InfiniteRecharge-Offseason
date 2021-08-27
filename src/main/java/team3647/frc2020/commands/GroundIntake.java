package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Intake;

public class GroundIntake extends CommandBase {
    private final Intake m_intake;

    /**
     * Creates a new GroundIntake.
     */
    public GroundIntake(Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.extendOuter();
        m_intake.extendInner();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.extendOuter();
        m_intake.extendInner();
        m_intake.intake(.7);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.intake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
