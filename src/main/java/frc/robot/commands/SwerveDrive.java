// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends Command {
  XboxController driveController;
  Drivetrain m_SwerveSubsystem;
  /** Creates a new SwerveDrive. */
  public SwerveDrive(Drivetrain m_SwerveSubsystem) {
    addRequirements(m_SwerveSubsystem);
    this.m_SwerveSubsystem = m_SwerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveSubsystem.driveWithJoystick(true);
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
