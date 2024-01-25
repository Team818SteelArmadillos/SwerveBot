// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class VisionController extends Command {
  Vision m_vision;
  /** Creates a new VisionController. */
  public VisionController(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vision.updatePoseLimelight();
    RobotContainer.m_SwerveSubsystem.resetOdometry(m_vision.getFusedPose());
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
