/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GroundIntakeSequence extends SequentialCommandGroup {
    /**
     * Creates a new GroundIntakeSequence.
     */
    public GroundIntakeSequence(Intake intake, Indexer indexer, BallStopper ballStopper) {
        super(new RunCommand(() -> {
            intake.retractInner();
        }, intake).withTimeout(.5), new ExtendIntakeToGround(intake).withTimeout(.25),
                new ParallelCommandGroup(new GroundIntake(intake), new LoadBalls(indexer, ballStopper)));
    }
}
