/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.PID_Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PID_ShooterCommand extends CommandBase {
  /**
   * Creates a new PID_ShooterCommand.
   */

   private PID_Shooter mShooter;
   private double distanceToTarget; 


  public PID_ShooterCommand(PID_Shooter tShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = tShooter;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceToTarget = (Constants.GameAttributes.targetHeight - Constants.RobotAttributes.limeHeight) / 
      Math.tan((NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0) * (Math.PI / 180)
       + Constants.RobotAttributes.limeTilt));
    
    double velocity = (distanceToTarget / Math.cos(Constants.RobotAttributes.shooterAngle)) * Math.sqrt(9.80665 / 
      (2*Math.tan(Constants.RobotAttributes.shooterAngle)*distanceToTarget + (Constants.RobotAttributes.shooterEndHeight - Constants.GameAttributes.targetHeight)));

    mShooter.manualShoot(velocity * Constants.ShooterMotors.sparksSpeedFactor);
    mShooter.setSetpoint(velocity * Constants.ShooterMotors.sparksSpeedFactor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mShooter.enable();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
