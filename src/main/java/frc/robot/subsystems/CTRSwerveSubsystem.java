// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.CTRSwerve.SwerveDriveConstantsCreator;
import frc.robot.CTRSwerve.SwerveDriveTrainConstants;
import frc.robot.CTRSwerve.SwerveModuleConstants;

public class CTRSwerveSubsystem extends SubsystemBase {

  private SwerveDriveTrainConstants drivetrain;
  private SlotConfiguration steerGains;
  private SlotConfiguration driveGains;
  private SwerveDriveConstantsCreator m_constantsCreator;
  private SwerveModuleConstants frontRight;
  private SwerveModuleConstants frontLeft;
  private SwerveModuleConstants backRight;
  private SwerveModuleConstants backLeft;
  private CTRSwerveDrivetrain m_drivetrain;
public Supplier<Pose2d> getPose2d;

  /** Creates a new CTRESwerveSubsystem. */
  public CTRSwerveSubsystem() {
    //CTRE Swerve
    drivetrain = new SwerveDriveTrainConstants().withPigeon2Id(Constants.pigeonID);
    drivetrain.withTurnKp(Constants.DRIVE_GAINS_KP);
    drivetrain.withTurnKd(Constants.DRIVE_TURN_KD);

    steerGains = new SlotConfiguration();
      steerGains.kP = Constants.STEER_GAINS_KP;
      steerGains.kD = Constants.STEER_GAINS_KD;

    driveGains = new SlotConfiguration();
      driveGains.kP = Constants.DRIVE_GAINS_KP;
      
    m_constantsCreator = new SwerveDriveConstantsCreator(
      Constants.DriveMotorGearRatio, 
      Constants.AZIMUTH_GEAR_RATIO, 
      Constants.WheelRadius, 
      0, //Constants.SLIP_CURRENT, 
      steerGains, 
      driveGains, 
      false
      );

    frontRight = m_constantsCreator.createModuleConstants(
      Constants.frontRightRotatePort, 
      Constants.frontRightDrivePort, 
      0, //Constants.FRONT_RIGHT_CANCODER, 
      0, //Constants.FRONT_RIGHT_OFFSET, 
      Constants.frameSize/2.0, 
      -Constants.frameSize/2.0
      );
    //Here
    frontLeft = m_constantsCreator.createModuleConstants(
      Constants.frontLeftRotatePort, 
      Constants.frontLeftDrivePort, 
      0, //Constants.FRONT_LEFT_CANCODER, 
      0, //Constants.FRONT_LEFT_OFFSET, 
      Constants.frameSize/2.0, 
      Constants.frameSize/2.0);

    backRight = m_constantsCreator.createModuleConstants(
      Constants.backRightRotatePort, 
      Constants.backRightDrivePort, 
      0, //Constants.BACK_RIGHT_CANCODER, 
      Conversions.tickstoDegrees(428, 1), //Constants.BACK_RIGHT_OFFSET, 
      -Constants.frameSize/2.0, 
      -Constants.frameSize/2.0);

    backLeft = m_constantsCreator.createModuleConstants(
      Constants.backLeftRotatePort, 
      Constants.backLeftDrivePort, 
      0, //Constants.BACK_LEFT_CANCODER, 
      0, //Constants.BACK_LEFT_OFFSET, 
      -Constants.frameSize/2.0, 
      Constants.frameSize/2.0);

    m_drivetrain = new CTRSwerveDrivetrain(drivetrain, frontRight, frontLeft, backRight, backLeft);

    m_drivetrain.seedFieldRelativeButBackwards(); // WE START BACKWARDS
  }

  public CTRSwerveDrivetrain getCTRSwerveDrivetrain() {
    return m_drivetrain;
  }

  public void setPose(Pose2d pose){
    m_drivetrain.setPose(pose);
  }

  public void setChasisSpeeds(ChassisSpeeds speeds){
    m_drivetrain.driveRobotCentric(speeds);
  }
  
  public Pose2d getPose(){
    return m_drivetrain.getPoseMeters();
  }

  public void resetPose() {
    setPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void periodic() {
    // Logger.getInstance().recordOutput("OdometryPose", getPose());
  }
}
