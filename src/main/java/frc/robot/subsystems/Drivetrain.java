// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  final double deadZone = 0.05;

  int navXDebug = 0;

  SwerveModulePosition[] swerveModulePosition;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final SwerveModule m_frontLeft = 
      new SwerveModule(
          Constants.frontLeftDrivePort,
          Constants.frontLeftRotatePort,
          false);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          Constants.frontLeftDrivePort,
          Constants.frontLeftRotatePort,
          false);

  private final SwerveModule m_backLeft = 
      new SwerveModule(
          Constants.frontLeftDrivePort,
          Constants.frontLeftRotatePort,
          false);

  private final SwerveModule m_backRight = 
      new SwerveModule(
          Constants.frontLeftDrivePort,
          Constants.frontLeftRotatePort,
          false);

  

    private final Pigeon2 m_pigeon;

  private final SwerveDriveOdometry m_odometry;

  public Drivetrain() {
    m_odometry =
      new SwerveDriveOdometry(Constants.kinematics, new Rotation2d(0), 
      new SwerveModulePosition[]{new SwerveModulePosition(m_frontLeft.getMotorVelocity(), new Rotation2d(m_frontLeft.getTurningRadians())), 
            new SwerveModulePosition(m_frontRight.getMotorVelocity(), new Rotation2d(m_frontRight.getTurningRadians())),
            new SwerveModulePosition(m_backLeft.getMotorVelocity(), new Rotation2d(m_backLeft.getTurningRadians())),
            new SwerveModulePosition(m_backLeft.getMotorVelocity(), new Rotation2d(m_backLeft.getTurningRadians()))});
    m_pigeon = new Pigeon2(Constants.pigeonID);
    m_pigeon.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Deadzones
    if (Math.abs(xSpeed) <= deadZone)
    xSpeed = 0;
    if (Math.abs(ySpeed) <= deadZone)
    ySpeed = 0;
    if (Math.abs(rot) <= deadZone)
    rot = 0;  

    // Try to normalize joystick limits to speed limits
    //xSpeed *= Math.abs(xSpeed);
    //ySpeed *= Math.abs(ySpeed);      
    //rot *= Math.abs(rot);

    var swerveModuleStates = Constants.kinematics.toSwerveModuleStates(fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SmartDashboard.putNumber("Front Left Drive Desired State", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left Turn Desired State", swerveModuleStates[0].angle.getDegrees());

    SmartDashboard.putNumber("Front Right Drive Desired State", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Turn Desired State", swerveModuleStates[1].angle.getDegrees());
    
    SmartDashboard.putNumber("Back Left Drive Desired State", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Turn Desired State", swerveModuleStates[2].angle.getDegrees());

    SmartDashboard.putNumber("Back Right Drive Desired State", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Turn Desired State", swerveModuleStates[3].angle.getDegrees());

    SmartDashboard.putNumber("Pigeon Reading", m_pigeon.getRotation2d().getDegrees());

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
        m_odometry.update(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[]{m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_pigeon.getRotation2d(), swerveModulePosition, pose);

    m_frontLeft.setPose(pose);
    m_frontRight.setPose(pose);
    m_backLeft.setPose(pose);
    m_backRight.setPose(pose);

    /*for(int i = 0; i < mSwerveModules.length; i++) {
        mSwerveModules[i].setPose(pose);
        mSwerveModules[i].resetEncoders();
    }*/
  }
public Pose2d getPose() {
  return m_odometry.getPoseMeters();
  }
  public Pose2d[] getModulePoses() {
    Pose2d[] modulePoses = {
        m_frontLeft.getPose(),
        m_frontRight.getPose(),
        m_backRight.getPose(),
        m_backLeft.getPose()
    };
    return modulePoses;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    updateSmartDashboard();
  }

  public void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(OI.getDrive().getLeftY())
            * Constants.maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(OI.getDrive().getLeftX())
            * Constants.maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(OI.getDrive().getRightX())
            * Constants.maxRotation;

    SmartDashboard.putNumber("XSpeed", xSpeed);
    SmartDashboard.putNumber("YSpeed", ySpeed);
    SmartDashboard.putNumber("Rot", rot);


    drive(xSpeed, ySpeed, rot, fieldRelative);
  }

      /**
     * Returns the raw angle of the robot in degrees
     *
     * @return The angle of the robot
     */
  public double getRawGyroAngle() {
    return m_pigeon.getRotation2d().getDegrees();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Robot Angle",getRawGyroAngle());
    SmartDashboard.putNumber("Front Left "  + " Angle", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Left" + " Speed", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right"  + " Angle", m_frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Right" + " Speed", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left"  + " Angle", m_backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Left" + " Speed", m_backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right"  + " Angle", m_backRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackRight" + " Speed", m_backRight.getState().speedMetersPerSecond);
  }
    /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

}
