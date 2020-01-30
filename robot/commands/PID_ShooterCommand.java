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
    
    // Utilizes vision to calculate the approximate distance to the target.
    distanceToTarget = (Constants.GameAttributes.targetHeight - Constants.RobotAttributes.limeHeight) / 
      Math.tan((NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0) * (Math.PI / 180)
       + Constants.RobotAttributes.limeTilt));
    
    // Using voodoo Physics, gets the velocity the projectile needs to be launched at in order to hit the targer.
    double velocity = (distanceToTarget / Math.cos(Constants.RobotAttributes.shooterAngle)) * Math.sqrt(9.80665 / 
      (2*Math.tan(Constants.RobotAttributes.shooterAngle)*distanceToTarget + (Constants.RobotAttributes.shooterEndHeight - Constants.GameAttributes.targetHeight)));
    
    // Starts up the motor at the supposed speed value, which is then calibrated by the PID Controller during execute().
    mShooter.manualShoot(velocity * Constants.ShooterMotors.sparksSpeedFactor);
    mShooter.setSetpoint(velocity * Constants.ShooterMotors.sparksSpeedFactor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Starts the PID Controller, and maps output to shooter motors.
    mShooter.enable();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Resets PID Controller to its original state.
    mShooter.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
