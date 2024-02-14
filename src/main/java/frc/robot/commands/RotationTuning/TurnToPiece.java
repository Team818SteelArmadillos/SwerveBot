// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.RotationTuning;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class TurnToPiece extends Command {
  /** Creates a new TurnToPiece. */
  SwerveDrivetrain m_SwerveDrivetrain;
  PIDController turnPID;
  Vision m_Vision;
  public TurnToPiece(SwerveDrivetrain m_swerveDriveTrain, Vision m_Vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveDrivetrain = m_swerveDriveTrain;
    turnPID = new PIDController(0.0025, 0.00005, 0); //FROM 2023
    this.m_Vision = m_Vision;
    m_Vision.setObjectDetection();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var rotation = turnPID.calculate(m_SwerveDrivetrain.getAngle(), m_SwerveDrivetrain.getAngle() + m_Vision.objectOffset());
    m_SwerveDrivetrain.drive(new Translation2d(), rotation, isFinished(), isScheduled());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Vision.objectOffset() < 5;
  }
}
