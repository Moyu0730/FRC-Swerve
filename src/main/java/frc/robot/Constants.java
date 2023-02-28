// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class JoystickConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int leftStick_X = 0;
    public static final int leftStick_Y = 1;
    public static final int rightStick_X = 4;
    public static final int rightStick_Y = 5;
    public static final int trigger_L = 2;
    public static final int trigger_R = 3;
    public static final int btn_A = 1;
    public static final int btn_B = 2;
    public static final int btn_X = 3;
    public static final int btn_Y = 4;
    public static final int btn_LB = 5;
    public static final int btn_RB = 6;
    public static final int btn_LS = 9;  
    public static final int btn_RS = 10;

    public static final double kDeadband = 0.05;
  }

  public static final class ModuleConstants {
    // Adjust Here
      public static final double kWheelDiameterMeters = 0.1034;
      public static final double kThrottleGearRatio = 6.75 / 1.0; // 6.75 : 1.0
      public static final double kRotorGearRatio = 150.0 / 7.0 / 1.0; // 12.8 : 1.0
      public static final double kThrottleEncoderRot2Meter = kThrottleGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kRotorEncoderRot2Rad = kRotorGearRatio * 2 * Math.PI;
      public static final double kThrottleEncoderRPM2MeterPerSec = kThrottleEncoderRot2Meter / 60;
      public static final double kRotorEncoderRPM2RadPerSec = kRotorEncoderRot2Rad / 60;
      public static final double kPTurning = 0.009;
  }

  public static class MotorConstants {
    
    // Distance between right and left wheels
    public static final double kTrackWidthInMeter = 0.288798;
    
    // Distance between front and back wheels
    public static final double kWheelBaseInMeter = 0.288798;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d( kWheelBaseInMeter, -kTrackWidthInMeter ),
      new Translation2d( kWheelBaseInMeter, kTrackWidthInMeter ),
      new Translation2d( -kWheelBaseInMeter, -kTrackWidthInMeter ),
      new Translation2d( -kWheelBaseInMeter, kTrackWidthInMeter ));

    // Throttle Port
    public static final int kFrontLeftThrottlePort = 12;
    public static final int kFrontRightThrottlePort = 22;
    public static final int kRearRightThrottlePort = 32;
    public static final int kRearLeftThrottlePort = 42;

    // Rotor Port
    public static final int kFrontLeftRotorPort = 11;
    public static final int kFrontRightRotorPort = 21;
    public static final int kRearRightRotorPort = 31;
    public static final int kRearLeftRotorPort = 41;

    // Rotor Inverted
    public static final boolean kFrontLeftRotorReversed = false;
    public static final boolean kRearLeftRotorReversed = false;
    public static final boolean kFrontRightRotorReversed = false;
    public static final boolean kRearRightRotorReversed = false;

    // Throttle Inverted
    public static final boolean kFrontLeftThrottleReversed = false;
    public static final boolean kRearLeftThrottleReversed = false;
    public static final boolean kFrontRightThrottleReversed = false;
    public static final boolean kRearRightThrottleReversed = false;

    // Rotor Encoder Port
    public static final int kFrontLeftRotorEncoderPort = 1;
    public static final int kFrontRightRotorEncoderPort = 2;
    public static final int kRearRightRotorEncoderPort = 3;
    public static final int kRearLeftRotorEncoderPort = 4;

    // Rotor Offset Rad
    public static final double kFrontLeftRotorEncoderOffsetRad = 134.121 * Math.PI / 180;
    public static final double kRearLeftRotorEncoderOffsetRad = -41.309 * Math.PI / 180;
    public static final double kFrontRightRotorEncoderOffsetRad = 85.078 * Math.PI / 180;
    public static final double kRearRightRotorEncoderOffsetRad = 48.076 * Math.PI / 180;

    // Physical Max Speed and Acceleraiton (/PerSec)
    public static final double kPhysicalMaxSpeedMetersPerSecond =  16 / 3.2808; // Meter = Feet / 3.2808
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    // Teleop Max Speed and Acceleration
    public static final double kTeleopThrottleMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleopThrottleMaxAngularAccelerationUnitsPerSecond = 3;
    public static final double kTeleopThrottleMaxAngluarSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleopThrottleMaxSpeedMetersPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4; 
  }
}
