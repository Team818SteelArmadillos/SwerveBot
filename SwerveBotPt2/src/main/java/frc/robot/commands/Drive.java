// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Swerve;

public class Drive extends Command {
  Swerve m_Swerve;
  /** Creates a new Drive. */
  public Drive(Swerve m_Swerve) {
    addRequirements(m_Swerve);

    this.m_Swerve = m_Swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XboxController driveXbox = OI.getDrive();
    var leftX = driveXbox.getLeftX();
    var leftY = driveXbox.getLeftY();
    var rightX = driveXbox.getRightX();

    if(Math.abs(leftX) < 0.3){
      leftX = 0;
    }
    if(Math.abs(leftY) < 0.3){
      leftY = 0;
    }
    if(Math.abs(rightX) < 0.3){
      rightX = 0;
    }
    m_Swerve.drive(-leftX, -leftY, -rightX);
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
