/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PID_DriveCommand extends PIDCommand {
  /**
   * Creates a new PID_DriveCommand.
   */

  Timer mTimer;

  public PID_DriveCommand(double time, double distance, DriveSubsystem drive) {
    double distance = Units.inchesToMeters(distance);
    
    super(
        // The controller that the command will use
        new PIDController(Constants.PID_DriveConstatmts. kP, Constants.PID_DriveConstatmts. kI, Constants.PID_DriveConstatmts. kD),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          drive.arcadeDrive(output);
        },
        drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    mTimer = new Timer();
    mTimer.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mTimer.get() > time) {return true;}
    else {return false;}
  }
}
