// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Joystick IDs
  public static class JoystickControllerConstants {
    // Joystick IDs
    public static final int kDriverControllerID = 0;

  }

  public static class MotorConstants {
    // Motor IDs

    public static final int kFrontLeftDriveMotor = 12;
    public static final int kFrontLeftAngleMotor = 13;

    public static final int kFrontRightDriveMotor = 3;
    public static final int kFrontRightAngleMotor = 2;

    public static final int kRearLeftDriveMotor = 15;
    public static final int kRearLeftAngleMotor = 14;

    public static final int kRearRightDriveMotor = 1;
    public static final int kRearRightAngleMotor = 0;

    public static final double kTranslationSlew = 1.55;
    public static final double kRotationSlew = 3.00;

    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
    public static final double kMaxSpeedMetersPerSecond = 3.25; // Maximum Sustainable Drivetrain Speed under Normal
    public static final double kMinRotationCommand = MotorConstants.kMaxAngularSpeed;
    public static final double kMinTranslationCommand = MotorConstants.kMaxSpeedMetersPerSecond;

    public static final double[] kFrontLeftTuningVals = { 0.0150, 0.2850, 0.25, 0 }; // {Static Gain, FeedForward,
      // Proportional Gain, ModuleID for
      // Tuning}
public static final double[] kFrontRightTuningVals = { 0.0150, 0.2850, 0.25, 1 }; // {Static Gain, FeedForward,
       // Proportional Gain, ModuleID for
       // Tuning}
public static final double[] kRearLeftTuningVals = { 0.0150, 0.2850, 0.25, 2 }; // {Static Gain, FeedForward,
     // Proportional Gain, ModuleID for
     // Tuning}
public static final double[] kRearRightTuningVals = { 0.0150, 0.2850, 0.25, 3 }; // {Static Gain, FeedForward,
      // Proportional Gain, ModuleID for
      // Tuning}


    
    public static final double[] kKeepAnglePID = { 0.500, 0, 0 }; // Defines the PID values for the keep angle PID

    public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
    // read when not in use (Eliminates "Stick Drift")
  public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
    // when aimed at a 45deg angle (Such that X and Y are are
    // maximized simultaneously)
  
}

public static final class ModuleConstants {
  public static final double kTranslationRampRate = 3.0; // Units of %power/s, ie 4.0 means it takes 0.25s to reach
                                                         // 100% power from 0%
  private static final double kTranslationGearRatio = 8.33333333; // Overall gear ratio of the swerve module
  private static final double kWheelDiameter = 0.0986 * 0.960; // Wheel Diameter in meters, may need to be
                                                                       // experimentally determined due to compliance
                                                                       // of floor/tread material

  public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; // Calculates
                                                                                                                // the
                                                                                                                // conversion
                                                                                                                // factor
                                                                                                                // of
                                                                                                                // RPM
                                                                                                                // of
                                                                                                                // the
                                                                                                                // translation
                                                                                                                // motor
                                                                                                                // to
                                                                                                                // m/s
                                                                                                                // at
                                                                                                                // the
                                                                                                                // floor

  // NOTE: You shoulds ALWAYS define a reasonable current limit when using
  // brushless motors
  // due to the extremely high stall current avaialble

  public static final double[] kTurnPID = { 0.600, 0, 0 }; // Defines the PID values for rotation of the serve
                                                           // modules, should show some minor oscillation when no
                                                           // weight is loaded on the modules
  public static final double kWheelCircumference = .3189;

  public static final double kDriveMotorGearRatio = 0.0935;
  public static final double kAngleMotorGearRatio = 0.0648;
}

  // Swerve Drive values
  // public static class SwerveConstants {
  //   public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
  //   public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 6 * Math.PI;
  // }
}
