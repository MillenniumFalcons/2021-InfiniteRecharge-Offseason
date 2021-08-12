package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2020.subsystems.Intake;

public class BetterLoadingStation extends CommandBase {
    private final Intake intake;
    public BetterLoadingStation(Intake intake) {
        this.intake = intake; 
        addRequirements(intake);
   }

   @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        intake.extendOuter();
        intake.extendInner();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}