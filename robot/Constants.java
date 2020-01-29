/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // PID Drive Constants

    public static class RobotAttributes {
        public static final short limeHeight = 0;
        public static final short limeTilt = 0;
        public static final short shooterAngle = 0;
        public static final short shooterEndHeight = 0;
           

    }

    public static class GameAttributes {
        public static final short targetHeight = 0;
        public static final short shootingTime = 0.5;
    }

    public static class PID_DriveConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }
    
    public static class PID_ShootConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }
    
    // Motor Ports
    public static class MotorPorts {
        public static final short pRight = 0;
        public static final short pLeft = 0;
        public static final short psRight = 0;
        public static final short psLeft = 0;
    }

    // Else
    public static class Else {
        public static short driveGoal;
        public static short trackWidth;
    }

    public static class MotorVelocity {
        public static short sRight;
        public static short sLeft;
    }

    public static class FeedShooter {
        public static short feedRight;
        public static short feedLeft;
        public static boolean rightInversion;
        public static boolean leftInversion;
        public static double feederSpeed;
    }

    public static class ShooterMechanism {
        public static short shootRight;
        public static short shootleft;
        public static boolean rightInversion;
        public static boolean leftInversion;
    }

    public static class ShooterEncoder {
        public static short aPort;
        public static short bPort;
        public static boolean inversion;
    }

    public static class 
}
