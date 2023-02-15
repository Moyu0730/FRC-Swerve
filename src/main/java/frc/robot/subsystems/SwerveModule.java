// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PIDConstants;

public class SwerveModule extends SubsystemBase {
  // Initialize Rotor & Throttle Motor
  private CANSparkMax mRotor;
  private CANSparkMax mThrottle;

  // Initialize Encoder
  private WPI_CANCoder mThrottleEncoder;
  private WPI_CANCoder mRotorEncoder;

  // Initialize Rotor PID Controller
  private PIDController mRotorPID;

  /** Creates a new ExampleSubsystem. */
  /**
   * 
   * @param throttleID
   * @param throttleEncoderID
   * @param rotorID
   * @param rotorEncoderID
   * @param rotorOffsetAngelDeg
   */
  public SwerveModule( int throttleID, int throttleEncoderID, int rotorID, int rotorEncoderID, double rotorOffsetAngleDeg ) {
    mRotor = new CANSparkMax(rotorID, MotorType.kBrushless);
    mThrottle = new CANSparkMax(throttleID, MotorType.kBrushless);
    
    mRotorEncoder = new WPI_CANCoder(rotorEncoderID);
    mThrottleEncoder = new WPI_CANCoder(throttleEncoderID);

    mThrottle.restoreFactoryDefaults();
    mRotor.restoreFactoryDefaults();
    mRotorEncoder.configFactoryDefault();

    mRotor.setInverted(MotorConstants.kRotorMotorInversion);
    mRotor.enableVoltageCompensation(Constants.kVoltageCompensation);
    mRotor.setIdleMode(IdleMode.kBrake);

    mRotorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    mRotorEncoder.configMagnetOffset(rotorOffsetAngleDeg);
    mRotorEncoder.configSensorDirection(MotorConstants.kRotorEncoderDirection);
    mRotorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    mRotorPID = new PIDController(
      PIDConstants.kRotor_kP,
      PIDConstants.kRotor_kI,
      PIDConstants.kRotor_kD
    );

    mRotorPID.enableContinuousInput(-180, 180);

    mThrottle.enableVoltageCompensation(Constants.kVoltageCompensation);
    mThrottle.setIdleMode(IdleMode.kBrake);

    // mThrottleEncoder.setVelocityConversionFactor( DriveConstants.kSparkThrottleVelocityConversionFactor );
  }

  /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(
          mThrottleEncoder.getVelocity(),
          Rotation2d.fromDegrees(mRotorEncoder.getAbsolutePosition())
      );
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        mRotorEncoder.getPosition(), new Rotation2d(mThrottleEncoder.getPosition()));
  }

  /**
     * Set module state
     * 
     * @param state module state 
     */
    public void setState(SwerveModuleState state) {
      // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
      SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
      
      // 通過比較目前角度與目標角度來用 PID 控制器計算轉向馬達所需的輸出
      double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

      mRotor.set(rotorOutput);
      mThrottle.set(optimizedState.speedMetersPerSecond);
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
