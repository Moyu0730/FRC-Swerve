// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Swerve extends SubsystemBase {

  private final SwerveModule mLeftFrontModule, mRightFrontModule, mLeftRearModule, mRightRearModule;
  private AHRS mGyro = new AHRS(SPI.Port.kMXP);
  private SwerveDriveOdometry mOdometry;

  /** Creates a new Swerve. */
  public Swerve() {
    mLeftFrontModule = new SwerveModule(
            MotorConstants.kLeftFrontThrottleID, 
            MotorConstants.kLeftFrontThrottleEncoderID, 
            MotorConstants.kLeftFrontRotorID, 
            MotorConstants.kLeftFrontRotorEncoderID, 
            MotorConstants.kLeftFrontRotorOffset
        );

        mRightFrontModule = new SwerveModule(
          MotorConstants.kRightFrontThrottleID, 
          MotorConstants.kRightFrontThrottleEncoderID, 
          MotorConstants.kRightFrontRotorID, 
          MotorConstants.kRightFrontRotorEncoderID, 
          MotorConstants.kRightFrontRotorOffset
        );

        mLeftRearModule = new SwerveModule(
          MotorConstants.kLeftRearThrottleID, 
          MotorConstants.kLeftRearThrottleEncoderID, 
          MotorConstants.kLeftRearRotorID, 
          MotorConstants.kLeftRearRotorEncoderID, 
          MotorConstants.kLeftRearRotorOffset
        );

        mRightRearModule = new SwerveModule(
          MotorConstants.kRightRearThrottleID, 
          MotorConstants.kRightRearThrottleEncoderID, 
          MotorConstants.kRightRearRotorID, 
          MotorConstants.kRightRearRotorEncoderID, 
          MotorConstants.kRightRearRotorOffset
        );

        mOdometry = new SwerveDriveOdometry(MotorConstants.kSwerveKinematics, mGyro.getRotation2d(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Updates odometry with current module state
    mOdometry.update(
        mGyro.getRotation2d(), 
        new SwerveModulePosition[] {
          getModulePositions()[0], 
          getModulePositions()[1],
          getModulePositions()[2],
          getModulePositions()[3]
        }
    );
  }

  /**
     * Get current swerve module states
     * 輸出 4 個 Swerve Module 的當前狀態 modules
     * 
     * @return swerve module states
     */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
        mLeftFrontModule.getState(), 
        mRightFrontModule.getState(), 
        mLeftRearModule.getState(), 
        mRightRearModule.getState()
    };
  }

  /**
     * Get current swerve module positions
     * 輸出 4 個 Swerve Module 的當前狀態 modules
     * 
     * @return swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[]{
          mLeftFrontModule.getPosition(), 
          mRightFrontModule.getPosition(), 
          mLeftRearModule.getPosition(), 
          mRightRearModule.getPosition()
      };
    }

  /**
     * Sets swerve module states
     * 設置 4 個 Swerve module 的狀態。
     * 
     * @param desiredStates array of desired states, order: [leftFront, leftRear, rightFront, rightRear]
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
      mLeftFrontModule.setState(desiredStates[0]);
      mRightFrontModule.setState(desiredStates[1]);
      mLeftRearModule.setState(desiredStates[2]);
      mRightRearModule.setState(desiredStates[3]);
  }

  /**
     * Drives the swerve - Input range: [-1, 1]
     * 
     * @param xSpeed percent power in the X direction (X 方向的功率百分比)
     * @param ySpeed percent power in the Y direction (Y 方向的功率百分比)
     * @param zSpeed percent power for rotation (旋轉的功率百分比)
     * @param fieldOriented configure robot movement style (設置機器運動方式) (field or robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
      SwerveModuleState[] states = null;
      if(fieldOriented) {
          states = MotorConstants.kSwerveKinematics.toSwerveModuleStates(
              // IMU used for field oriented control
              // IMU 用於 Field Oriented Control
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, mGyro.getRotation2d())
          );
      } else {
          states = MotorConstants.kSwerveKinematics.toSwerveModuleStates(
              new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
          );
      }
      setModuleStates(states);
  }

  /**
     * Get predicted pose
     * 獲取機器人的當前位置
     * 
     * @return pose
     */
    public Pose2d getPose() {
      return mOdometry.getPoseMeters();
  }

  /**
     * Set robot pose
     * 將測程法（odometry）位置設置為給與的 x、y、位置和角度
     * 
     * @param pose robot pose
     */
    public void setPose(Pose2d pose, SwerveModulePosition[] modulepos ) {
      mOdometry.resetPosition(mGyro.getRotation2d(), modulepos, pose);
  }
}
