/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

   private final MotorControllerGroup fRight = new MotorControllerGroup(new Talon(Constants.MotorPorts.mRight), new Talon(Constants.MotorPorts.sRight));
   private final MotorControllerGroup fLeft = new MotorControllerGroup(new Talon(Constants.MotorPorts.mLeft), new Talon(Constants.MotorPorts.sLeft));

   DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.Else.trackWidth));
   var wheelSpeeds = new DifferentialDriveWheelSpeeds(Constants.MotorVelocity.vRight, Constants.MotorVelocity.vLeft);

   public final DifferentialDrive mDrive;
   
  public DriveSubsystem() {
    mDrive =  = new DifferentialDrive (fRight, fLeft);

  }

  public void arcadeDrive (double forward, double rotate) {
    mDrive.arcadeDrive(forward, rotate);
  }

  public void autoDrive(double distance, double time) {
    double forward = distance / time;
    mDrive.arcadeDrive(forward, 0);
  }

  public double getNiceSpeed () {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
