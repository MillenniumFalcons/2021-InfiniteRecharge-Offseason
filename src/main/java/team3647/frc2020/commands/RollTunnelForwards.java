/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Indexer;
import team3647.lib.IndexerSignal;

public class RollTunnelForwards extends CommandBase {

    private final Indexer m_indexer;
  /**
   * Creates a new RollIndexerBack.
   */
  public RollTunnelForwards(Indexer indexer) {
      m_indexer = indexer;
      addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_indexer.set(IndexerSignal.INDEXERFWD_SLOW);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_indexer.set(IndexerSignal.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_indexer.getBannerSensorValue();
  }
}