/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class PID_Shooter extends PIDSubsystem {
  /**
   * Creates a new PID_Shooter.
   */

   private Spark feedRight = new Spark (Constants.FeederMotors.rightPort);
   private Spark feedLeft = new Spark (Constants.FeederMotors.leftPort);

   private Spark shootRight = new Spark (Constants.ShooterMotors.rightPort);
   private Spark shootLeft = new Spark (Constants.ShooterMotors.leftPort);

   private Encoder checkSpeed = new Encoder (Constants.ShooterEncoder.aChannelPort, 
    Constants.ShooterEncoder.bChannelPort, Constants.ShooterEncoder.inversion);

  public PID_Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.PID_ShooterConstants.kP, 
          Constants.PID_ShooterConstants.kI, Constants.PID_ShooterConstants.kD));
          
    feedRight.setInverted (Constants.FeederMotors.rightInverted);
    feedLeft.setInverted (Constants.FeederMotors.leftInverted);
  
    shootRight.setInverted (Constants.ShooterMotors.rightInverted);
    shootLeft.setInverted (Constants.ShooterMotors.leftInverted);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    shootRight.setSpeed(output);
    shootLeft.setSpeed(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return checkSpeed.getRate();
  }

  public void manualShoot (double speed) {
    shootRight.setSpeed(speed);
    shootLeft.setSpeed(speed);
  }

  public void manualFeed (double speed) {
    feedRight.setSpeed(speed);
    feedLeft.setSpeed(speed);
  }
}
