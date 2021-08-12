/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.Intake;
import team3647.frc2020.subsystems.KickerWheel;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StowIntakeAndOrganizeFeeder extends SequentialCommandGroup {
    /**
     * Creates a new StowIntakeAndOrganizeFeeder.
     */
    public StowIntakeAndOrganizeFeeder(Intake intake, Indexer indexer, KickerWheel kickerWheel) {
         super(new RunCommand(intake::end, intake).withTimeout(.1), new StowOuterExtendInner(intake).withTimeout(.1), new WaitCommand(1), 
                 new OrganizeFeeder(indexer, kickerWheel).withTimeout(1));
        
    }
}
