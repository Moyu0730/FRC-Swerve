// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ManualDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_Swerve;
  private final Joystick m_DriverJoystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualDrive(Swerve drive, Joystick controller) {
    m_Swerve = drive;
    m_DriverJoystick = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Swerve.drive(m_DriverJoystick.getRawAxis( JoystickConstants.leftStick_Y ) * 0.7,
                   m_DriverJoystick.getRawAxis( JoystickConstants.leftStick_X ) * 0.7,
                   m_DriverJoystick.getRawAxis( JoystickConstants.rightStick_X ) * 0.7, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
