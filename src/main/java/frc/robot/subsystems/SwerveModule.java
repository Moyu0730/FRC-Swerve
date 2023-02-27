// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.MotorConstants;
import frc.robot.library.PID;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  // Initialize Rotor & Throttle Motor
  private final CANSparkMax m_Throttle;
  private final CANSparkMax m_Rotor;

  // Initialize Throttle Encoder Position
  // private final double m_ThrottleEncoderPosition;

  // Initialize Rotor PID Controller
  // private final PIDController mTurningPIDController;
  private final PID mRotorPID;

  // Initialize Rotor(Absolute) Encoders
  private final WPI_CANCoder m_RotorEncoder;
  private final double m_RotorEncoderOffsetRad;

  /** Creates a new ExampleSubsystem. */
  /**
   * 
   * @param ThrottleID
   * @param RotorID
   * @param ThrottleReversed
   * @param RotorReversed
   * @param RotorEncoderID (CANCoder)
   * @param RotorEncoderOffset
   * @param RotorEncoderReversed
   */
  public SwerveModule( int throttleID, int rotorID, boolean throttleReversed, boolean rotorReversed, int rotorEncoderID, double rotorEncoderOffset ) {
    this.m_RotorEncoderOffsetRad = rotorEncoderOffset;

    m_RotorEncoder = new WPI_CANCoder(rotorEncoderID);

    m_Throttle = new CANSparkMax(throttleID, MotorType.kBrushless);
    m_Rotor = new CANSparkMax(rotorID, MotorType.kBrushless);

    m_Throttle.setInverted(throttleReversed);
    m_Rotor.setInverted(rotorReversed);

    m_Throttle.getEncoder().setPositionConversionFactor(ModuleConstants.kThrottleEncoderRot2Meter);
    m_Throttle.getEncoder().setVelocityConversionFactor(ModuleConstants.kThrottleEncoderRPM2MeterPerSec);
    // m_RotorEncoder.setPositionConversionFactor(ModuleConstants.kRotorEncoderRot2Rad);
    // m_RotorEncoder.setVelocityConversionFactor(ModuleConstants.kRotorEncoderRPM2RadPerSec);
    m_RotorEncoder.configFactoryDefault();

    // mTurningPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    // mTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    mRotorPID = new PID(ModuleConstants.kPTurning, 0, 0, 0);
    mRotorPID.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  public double getThrottleEncoderPosition() {
    return m_Throttle.getEncoder().getPosition();
  }

  public double getRotorEncoderPosition() {
    return m_RotorEncoder.getPosition();
  }

  public double getThrottleEncoderVelocity() {
    return m_Throttle.getEncoder().getVelocity();
  }

  public double getRotorEncoderVelocity() {
    return m_RotorEncoder.getVelocity();
  }

  public double getRotorEncoderRad() {
    return m_RotorEncoder.getAbsolutePosition() * Math.PI / 180; // Angle
    // return angle * ( m_RotorEncoderReversed ? -1 : 1 );
  }

  public void resetEncoders() {
    m_Throttle.getEncoder().setPosition(0);
    m_RotorEncoder.setPosition(getRotorEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getThrottleEncoderVelocity(), new Rotation2d(getRotorEncoderPosition()));
  }

  public void setDesiredstate( SwerveModuleState state ){
    if( Math.abs(state.speedMetersPerSecond) < 0.001 ){
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    m_Throttle.set(state.speedMetersPerSecond / MotorConstants.kPhysicalMaxSpeedMetersPerSecond);
    m_Rotor.set(mRotorPID.calculate(getRotorEncoderPosition(), state.angle.getRadians()));

    SmartDashboard.putString("Swerve [" + m_RotorEncoder.getDeviceID() + "] State", state.toString());
  }

  public void stop(){
    m_Throttle.set(0);
    m_Rotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
