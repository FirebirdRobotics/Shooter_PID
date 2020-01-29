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
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class PID_DriveSubsystem extends PIDSubsystem {
  /**
   * Creates a new PID_DriveSubsystem.
   */

  Talon right = new Talon (Constants.pRight), left = new Talon (Constants.pRight), sRight = new Talon(Constants.psRight), sLeft = new Talon(Constants.psLeft);

  SpeedControllerGroup mRight = new SpeedControllerGroup(right, sRight);
  SpeedControllerGroup mLeft = new SpeedControllerGroup (right, sRight);
  DifferentialDrive dDrive = new DifferentialDrive(mRight, mLeft);
  int output_s;
  this.setSetpoint(RobotContainer.myController.getX(GenericHID.Hand.kLeft))

  public PID_DriveSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.PID_DriveConstants.kP, Constants.PID_DriveConstants.kI, Constants.PID_DriveConstants.kD));
    
  }

  @Override
  public void useOutput(double output, double setpoint) {
    output_s = output;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public void arcadeDrive() {
    
  }
}
