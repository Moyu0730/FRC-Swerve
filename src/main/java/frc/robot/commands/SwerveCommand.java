// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends CommandBase {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final Supplier<Double> m_xSpeedFunction, m_ySpeedFunction, m_turningSpeedFunction;
  private final Supplier<Boolean> m_FieldOrientedFunction;
  private final SlewRateLimiter m_xLimiter, m_yLimiter, m_turningLimiter;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveCommand( SwerveSubsystem swerveSubsystem,
                                Supplier<Double> xSpdFunciton, Supplier<Double> ySpdFunciton, Supplier<Double> turningSpdFunciton,
                                Supplier<Boolean> fieldOrientedFunciton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_xSpeedFunction = xSpdFunciton;
    this.m_ySpeedFunction = ySpdFunciton;
    this.m_turningSpeedFunction = turningSpdFunciton;
    this.m_FieldOrientedFunction = fieldOrientedFunciton;
    this.m_xLimiter = new SlewRateLimiter(MotorConstants.kTeleopThrottleMaxAccelerationUnitsPerSecond);
    this.m_yLimiter = new SlewRateLimiter(MotorConstants.kTeleopThrottleMaxAccelerationUnitsPerSecond);
    this.m_turningLimiter = new SlewRateLimiter(MotorConstants.kTeleopThrottleMaxAngularAccelerationUnitsPerSecond);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double m_xSpeed = m_xSpeedFunction.get();
    double m_ySpeed = m_ySpeedFunction.get();
    double m_turningSpeed = m_turningSpeedFunction.get();

    // 2. Apply DeadlBand
    m_xSpeed = Math.abs(m_xSpeed) > JoystickConstants.kDeadband ? m_xSpeed : 0;
    m_ySpeed = Math.abs(m_ySpeed) > JoystickConstants.kDeadband ? m_ySpeed : 0;
    m_turningSpeed = Math.abs(m_turningSpeed) > JoystickConstants.kDeadband ? m_turningSpeed : 0;

    // 3. Make the driving smoother
    // m_xSpeed = m_xLimiter.calculate(m_xSpeed) * MotorConstants.kTeleopThrottleMaxSpeedMetersPerSecond;
    // m_ySpeed = m_yLimiter.calculate(m_ySpeed) * MotorConstants.kTeleopThrottleMaxSpeedMetersPerSecond;
    // m_turningSpeed = m_turningLimiter.calculate(m_turningSpeed) * MotorConstants.kTeleopThrottleMaxAngluarSpeedRadiansPerSecond;

    // 4. Construct desired chassis speeds
    ChassisSpeeds m_ChassisSpeeds;
    if( m_FieldOrientedFunction.get() ){
      // Relative to field
      m_ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_turningSpeed, m_SwerveSubsystem.getRotation2d());
    } else {
      // Relative to robot
      m_ChassisSpeeds = new ChassisSpeeds(m_xSpeed, m_ySpeed, m_turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] m_ModuleStates = MotorConstants.kDriveKinematics.toSwerveModuleStates(m_ChassisSpeeds);

    // 6. Output each module states to wheels
    m_SwerveSubsystem.setModuleStates(m_ModuleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
