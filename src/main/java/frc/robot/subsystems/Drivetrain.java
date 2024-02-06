// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.MotorConstants;

import javax.swing.plaf.basic.BasicBorders.MenuBarBorder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Drivetrain extends SubsystemBase {

  private final PIDController m_keepAnglePID = new PIDController(MotorConstants.kKeepAnglePID[0],
  MotorConstants.kKeepAnglePID[1], MotorConstants.kKeepAnglePID[2]);

  private AHRS navx = new AHRS();

  private double keepAngle = 0.0; // Double to store the current target keepAngle in radians
  private double timeSinceRot = 0.0; // Double to store the time since last rotation command
  private double lastRotTime = 0.0; // Double to store the time of the last rotation command
  private double timeSinceDrive = 0.0; // Double to store the time since last translation command
  private double lastDriveTime = 0.0; // Double to store the time of the last translation command

  private final RRSwerveRipOff m_frontLeft = new RRSwerveRipOff(MotorConstants.kFrontLeftDriveMotor,
   MotorConstants.kFrontLeftAngleMotor, MotorConstants.kFrontLeftTuningVals);
  private final RRSwerveRipOff m_frontRight = new RRSwerveRipOff(MotorConstants.kFrontRightDriveMotor,
   MotorConstants.kFrontRightAngleMotor, MotorConstants.kFrontRightTuningVals);
  private final RRSwerveRipOff m_rearLeft = new RRSwerveRipOff(MotorConstants.kRearLeftDriveMotor,
   MotorConstants.kRearLeftAngleMotor, MotorConstants.kRearLeftTuningVals);
  private final RRSwerveRipOff m_rearRight = new RRSwerveRipOff(MotorConstants.kRearRightDriveMotor,
   MotorConstants.kRearRightAngleMotor, MotorConstants.kRearRightTuningVals);


  // Kinematics
  // The locations for the wheels must be relative to the center of the robot.
  // Positive x values represent moving toward the front of the robot whereas
  // positive y values represent moving toward the left of the robot.
  private Translation2d frontLeftTranslate = new Translation2d(0.3051, 0.2422);
  private Translation2d frontRightTranslate = new Translation2d(0.3051, -0.2422);
  private Translation2d rearLeftTranslate = new Translation2d(-0.3051, 0.2422);
  private Translation2d rearRightTranslate = new Translation2d(-0.3051, -0.2422);

  private final Timer keepAngleTimer = new Timer(); // Creates timer used in the perform keep angle function



  // Creates the kinematics object using the wheel locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    frontLeftTranslate, 
    frontRightTranslate, 
    rearLeftTranslate, 
    rearRightTranslate);

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MotorConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  
  // Creates the odometry object from the kinematics object and the initial wheel
  // positions.
  private Field2d field = new Field2d();



  private Pose2d robotPose =
    new Pose2d(0.0, 0.0, new Rotation2d());


    //TODO: figure out how to initialize the swervedrive odometry with module positions or without
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, navx.getRotation2d(),
     new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    }
    );

  // private SwerveDriveOdometry odometry =
  //   new SwerveDriveOdometry(m_kinematics, navx.getRotation2d(),
  //    new SwerveModulePosition[] {
  //     frontLeftModule
  //   }, robotPose);

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    rot = performKeepAngle(xSpeed, ySpeed, rot);
                                                     // depending on driver input

    // SmartDashboard.putNumber("xSpeed Commanded", xSpeed);
    // SmartDashboard.putNumber("ySpeed Commanded", ySpeed);

    // creates an array of the desired swerve module states based on driver command
    // and if the commands are field relative or not
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // normalize wheel speeds so all individual states are scaled to achievable
    // velocities
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MotorConstants.kMaxSpeedMetersPerSecond);

    setModuleStates(swerveModuleStates);
  }

  public Rotation2d getGyro() {
    return navx.getRotation2d();
  }

  public void updateOdometry() {
    m_odometry.update(navx.getRotation2d(), new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    });
  }  

  public ChassisSpeeds getChassisSpeed() {
    return m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }  

  private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot;
    if (Math.abs(rot) >= MotorConstants.kMinRotationCommand) {
      lastRotTime = keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= MotorConstants.kMinTranslationCommand || 
      Math.abs(ySpeed) >= MotorConstants.kMinTranslationCommand) {
        lastDriveTime = keepAngleTimer.get();
      }
    timeSinceRot = keepAngleTimer.get() - lastRotTime;
    timeSinceDrive = keepAngleTimer.get() - lastDriveTime;
    if (timeSinceRot < 0.5) {
      keepAngle = getGyro().getRadians();
    }
    else if (Math.abs(rot) < MotorConstants.kMinRotationCommand && timeSinceDrive < 0.25) {
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle);
    }
    return output;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateOdometry();
    }
}
