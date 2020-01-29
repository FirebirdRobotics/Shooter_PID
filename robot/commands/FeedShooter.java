/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {
  /**
   * Creates a new FeedShooter.
   */

  private PID_Shooter mShooter;
  private Timer mTimer;

  public FeedShooter(PID_Shooter tShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = tShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.manualFeed(Constants.FeedShooter.feederSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mTimer = 10) return true;
    else return false;
  }
}
