// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.FalconSpinnySubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FalconSpinny extends Command {
  private FalconSpinnySubsystem m_FalconSpinnySubsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FalconSpinny(FalconSpinnySubsystem m_FalconSpinnySubsystem) {
    addRequirements(m_FalconSpinnySubsystem);
    this.m_FalconSpinnySubsystem = m_FalconSpinnySubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_FalconSpinnySubsystem.setMotor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        m_FalconSpinnySubsystem.setMotor(-OI.getTest().getLeftY());
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
