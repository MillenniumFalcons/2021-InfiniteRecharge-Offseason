/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.Intake;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

public class RemoveBalls extends CommandBase {
    private final KickerWheel m_kickerWheel;
    private final Intake m_intake;
    private final Indexer m_indexer;
  /**
   * Creates a new RemoveBalls.
   */
  public RemoveBalls(Indexer indexer, Intake intake, KickerWheel kickerWheel) {
      m_kickerWheel = kickerWheel;
      m_intake = intake;
      m_indexer = indexer;
      addRequirements(m_kickerWheel, m_intake, m_indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_intake.retractInner();
      m_intake.extendOuter();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_intake.spitOut(1);
      m_indexer.set(IndexerSignal.SPITOUT);
      m_kickerWheel.setOpenloop(-.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.end();
    m_indexer.end();
    m_kickerWheel.end();
    m_intake.retractOuter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
