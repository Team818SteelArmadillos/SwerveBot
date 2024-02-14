// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.RotationTuning.TurnToPiece;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  public Vision m_Vision = new Vision();
  public SwerveDrivetrain swerveDrivetrain_subsystem = new SwerveDrivetrain();
  public XboxController xboxXontroller = new XboxController(0);
  public SwerveDrive swerveDrive_command = new SwerveDrive(
    swerveDrivetrain_subsystem, 
    xboxXontroller,
    true, 
    false, 
    m_Vision);

    private final TurnToPiece m_TurnToPiece = new TurnToPiece(swerveDrivetrain_subsystem, m_Vision);

  public RobotContainer() {
    swerveDrivetrain_subsystem.setDefaultCommand(swerveDrive_command);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return m_TurnToPiece;
  }
}
