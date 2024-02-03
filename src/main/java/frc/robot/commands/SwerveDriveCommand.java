// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.subsystems.CTRSwerveSubsystem;

public class SwerveDriveCommand extends CommandBase {

  private CTRSwerveDrivetrain m_drivetrain;
  private Rotation2d m_lastTargetAngle;
  private int drive_lock_counter;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(CTRSwerveSubsystem ctrSwerveSubsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ctrSwerveSubsytem);
    m_drivetrain = ctrSwerveSubsytem.getCTRSwerveDrivetrain();
    m_lastTargetAngle = new Rotation2d();
    drive_lock_counter = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drivetrain.seedFieldRelativeButBackwards(); // we start backwards so set yaw to 180
    m_lastTargetAngle = m_drivetrain.getPoseMeters().getRotation();
    drive_lock_counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = -OI.getDrive().getLeftY();
    double leftX = OI.getDrive().getLeftX();
    double rightX = OI.getDrive().getRightX();
    double speedFactor = Constants.maxSpeed * (1.0 - (OI.getDrive().getRightTriggerAxis()*0.9)); // right trigger 100% to 10% speed control
    double rotationFactor = Constants.maxRotation;

    if (Math.abs(leftY) < Constants.DriverDeadzone && Math.abs(leftX) < Constants.DriverDeadzone) {
        leftY = 0;
        leftX = 0;
    }

    if (Math.abs(rightX) < Constants.DriverDeadzone) {
        rightX = 0;
    }

    var directions = new ChassisSpeeds();
    directions.vxMetersPerSecond = leftY * speedFactor; // 17 ft per second
    directions.vyMetersPerSecond = leftX * -speedFactor; // 17 ft per second
    directions.omegaRadiansPerSecond = rightX * -rotationFactor;

    /* If we're pressing B, don't move, otherwise do normal movement */
    if (OI.getDrive().getBButton()) {
        m_drivetrain.driveStopMotion();
    } else {
        /* If we're fully field centric, we need to be pretty deflected to target an angle */
        if (OI.getDrive().getYButton()) { // orient forwards
          m_lastTargetAngle = new Rotation2d(1.0, 0.0);
        } else if (OI.getDrive().getAButton()) { // orient backwards
          m_lastTargetAngle = new Rotation2d(-1.0, 0.0);
        } else {
          // do nothing
        }

        if (Math.abs(rightX) > Constants.DriverDeadzone) {
          m_drivetrain.driveFieldCentric(directions);
          drive_lock_counter = 0;
        } else {
          drive_lock_counter++;
          if (drive_lock_counter >= 20) {
            m_drivetrain.driveFullyFieldCentric(leftY * speedFactor, leftX * -speedFactor, m_lastTargetAngle);
            drive_lock_counter = 20; // prevent overflow
          } else {
            m_drivetrain.driveFieldCentric(directions);
            m_lastTargetAngle = m_drivetrain.getPoseMeters().getRotation();
          }
        }


    }

    if (OI.getDrive().getXButton()) {
        m_drivetrain.seedFieldRelative();
        // Make us target forward now to avoid jumps
        m_lastTargetAngle = new Rotation2d();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveStopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
