// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MathUtils;

public class TeleDrive extends Command {
  /** Creates a new TeleDrive. */

  final Drivetrain m_drivetrain;
  final XboxController m_controller;

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(MotorConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(MotorConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(MotorConstants.kRotationSlew);

  final boolean fieldOrient = true;

  public TeleDrive(Drivetrain drivetrain, XboxController controller) {
    m_drivetrain = drivetrain;
    m_controller = controller;
    
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_slewX.calculate(-inputTransform(m_controller.getLeftY()))
    * MotorConstants.kMaxSpeedMetersPerSecond,
    m_slewY.calculate(-inputTransform(m_controller.getLeftX()))
    * MotorConstants.kMaxSpeedMetersPerSecond, m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
    * MotorConstants.kMaxAngularSpeed, fieldOrient);

    SmartDashboard.putBoolean("DrivingByController", true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  SmartDashboard.putBoolean("DrivingByController", false);

  }

  private double inputTransform(double input) {
    return MathUtils.singedSquare(MathUtils.applyDeadband(input));
  }

}
