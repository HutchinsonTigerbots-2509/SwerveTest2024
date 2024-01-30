// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.*;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class RRSwerveRipOff {
    private final TalonFX m_driveMotor;
    private final TalonFX m_rotationMotor;

    private final PIDController m_drivePIDController;

    //Create SimpleMotorFeedForward for the translation motor on the swerve module
    //The static and feedforward gains should be passed into the class constructor vial the "tuningCals" array
    private SimpleMotorFeedforward driveFeedForward;

    //Create a PIDController for the control of the angular position of the swerve module
    private final PIDController m_rotationPIDController = new PIDController(ModuleConstants.kTurnPID[0], ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, and drive encoder
     */
    public RRSwerveRipOff(int translationMotorChannel, int rotationMotorChannel, double[] tuningVals){
        m_driveMotor = new TalonFX(translationMotorChannel);


        m_driveMotor.setInverted(false);
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

        m_rotationMotor = new TalonFX(rotationMotorChannel);
        m_rotationMotor.setInverted(false);
        m_rotationMotor.setNeutralMode(NeutralModeValue.Brake);

        // You don't need to initialize seperate encoders for the TalonFX integrated encoders!
        

        //Set up you rotation encoder too, because I don't want to look that up either

        //Create SImpleMotorFeedForward for swerve module using the static and feedforward gains from the tuningVals array
        SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(tuningVals[0], tuningVals[1]);

        //Creates the drive PIDController using the proportional gain from the tuningVals array
        m_drivePIDController = new PIDController(tuningVals[2], 0.0, 0.0);

        // Sets the moduleID to the value stored in the tuningVals array
        final double moduleID = tuningVals[3];

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getVelocity().getValue(), new Rotation2d(getTurnEncoder()));

    }

    public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoder()));
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
    // Calculates the desired feedForward motor % from the current desired velocity
    // and the static and feedforward gains
    final double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);
    // Set the drive motor to the sum of the feedforward calculation and PID
    // calculation
    final double finalDriveOutput = driveOutput + driveFF;
    m_driveMotor.set(finalDriveOutput);
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_rotationPIDController.calculate(getTurnEncoder(), state.angle.getRadians());
    // Set the turning motor to this output value
    m_rotationMotor.set(turnOutput);
    // SmartDashboard.putNumber("TurnMotor"+moduleID, turnOutput);
  }

  public void stop() {
    m_driveMotor.set(0);
    m_rotationMotor.set(0);
  }

  public double getTurnEncoder() {
    return m_rotationMotor.getPosition().getValue();
  }

  private Rotation2d getAngle(){
    return Rotation2d.fromDegrees(MathUtils.falconToDegrees(m_rotationMotor.getPosition().getValue(), ModuleConstants.kAngleMotorGearRatio));
}

      public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            MathUtils.falconToMeters(m_driveMotor.getPosition().getValue(), ModuleConstants.kWheelCircumference, ModuleConstants.kDriveMotorGearRatio), 
            getAngle()
        );
    }

  public double getDriveVelocity() {
    return m_driveMotor.getVelocity().getValue();
  }
}