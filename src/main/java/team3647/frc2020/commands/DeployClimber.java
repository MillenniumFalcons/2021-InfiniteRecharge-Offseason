/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Climber;
import team3647.lib.wpi.Timer;

public class DeployClimber extends CommandBase {
    private final Climber m_climber;
    private final Timer timer;
  /**
   * Creates a new DeployClimber.
   */
  public DeployClimber(Climber climber) {
      m_climber = climber;
      timer = new Timer();
      addRequirements(m_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_climber.release();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_climber.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 2;
  }
}
