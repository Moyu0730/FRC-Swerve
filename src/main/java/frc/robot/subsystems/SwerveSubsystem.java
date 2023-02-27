// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;


public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule mFrontLeftModule 
                              = new SwerveModule(
                                      MotorConstants.kFrontLeftThrottlePort, 
                                      MotorConstants.kFrontLeftRotorPort, 
                                      MotorConstants.kFrontLeftThrottleReversed, 
                                      MotorConstants.kFrontLeftRotorReversed,
                                      MotorConstants.kFrontLeftRotorEncoderPort,
                                      MotorConstants.kFrontLeftRotorEncoderOffsetRad), 
                             mFrontRightModule 
                              = new SwerveModule(
                                      MotorConstants.kFrontRightThrottlePort, 
                                      MotorConstants.kFrontRightRotorPort, 
                                      MotorConstants.kFrontRightThrottleReversed,
                                      MotorConstants.kFrontRightRotorReversed,
                                      MotorConstants.kFrontRightRotorEncoderPort,
                                      MotorConstants.kFrontRightRotorEncoderOffsetRad),
                             mRearLeftModule 
                              = new SwerveModule(
                                      MotorConstants.kRearLeftThrottlePort, 
                                      MotorConstants.kRearLeftRotorPort, 
                                      MotorConstants.kRearLeftThrottleReversed, 
                                      MotorConstants.kRearLeftRotorReversed,
                                      MotorConstants.kRearLeftRotorEncoderPort,
                                      MotorConstants.kRearLeftRotorEncoderOffsetRad), 
                             mRearRightModule
                              = new SwerveModule(
                                      MotorConstants.kRearRightThrottlePort, 
                                      MotorConstants.kRearRightRotorPort, 
                                      MotorConstants.kRearRightThrottleReversed, 
                                      MotorConstants.kRearRightRotorReversed,
                                      MotorConstants.kRearRightRotorEncoderPort,
                                      MotorConstants.kRearRightRotorEncoderOffsetRad);
  private AHRS mGyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new Swerve. */
  public SwerveSubsystem() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      } catch ( Exception e ) {}
    }).start();
  }

  public void zeroHeading() {
    mGyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(mGyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
    mFrontLeftModule.stop();
    mFrontRightModule.stop();
    mRearLeftModule.stop();
    mRearRightModule.stop();
  }

  public void setModuleStates( SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MotorConstants.kPhysicalMaxSpeedMetersPerSecond);
    mFrontLeftModule.setDesiredstate(desiredStates[0]);
    mFrontRightModule.setDesiredstate(desiredStates[1]);
    mRearLeftModule.setDesiredstate(desiredStates[2]);
    mRearRightModule.setDesiredstate(desiredStates[3]);
  }
}
