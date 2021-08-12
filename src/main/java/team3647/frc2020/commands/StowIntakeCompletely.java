/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StowIntakeCompletely extends CommandBase {
    private final Intake m_intake;

    public StowIntakeCompletely(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.retractInner();
        m_intake.retractOuter();
        m_intake.end();
    }

    @Override
    public void execute() {
        initialize();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
