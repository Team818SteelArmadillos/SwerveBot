// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  
  private final WPI_TalonSRX m_driveMotor;
  private final WPI_TalonSRX m_turningMotor;

  Pose2d swerveModulePose = new Pose2d();

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController =
      new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.maxRotation,
              Math.pow(Constants.maxRotation, 2)));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = 
      new SimpleMotorFeedforward(
        Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter);
  private final SimpleMotorFeedforward m_turnFeedforward = 
      new SimpleMotorFeedforward(
        Constants.ksVolts,
        Constants.kvVoltSecondsPerRadian,
        Constants.kaVoltSecondsSquaredPerRadian);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param drivemotorinvert Whether or not the drive motor should be inverted
   * 
   */
  public SwerveModule(    
    int driveMotorChannel,
    int turningMotorChannel,
    boolean drivemotorinvert){
    m_driveMotor = new WPI_TalonSRX(driveMotorChannel);
    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);

    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_driveMotor.setInverted(drivemotorinvert);
    m_driveMotor.setSelectedSensorPosition(0);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.get()));
        //SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningRadians()));
    


    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        //m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
        m_drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningMotor.get(), state.angle.getRadians());
        //m_turningPIDController.calculate(getTurningRadians(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public void setPose(Pose2d pose) {
    swerveModulePose = pose;
  }

  /**
  * Returns the current velocity of the module.
  *
  * @return The current velocity of the module.
  */
  public double getVelocity() {
    return m_driveMotor.getSelectedSensorVelocity() * Constants.driveDistancePerPulse * 10;
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      (m_driveMotor.getSelectedSensorPosition() * ((Units.inchesToMeters(2) * Math.PI)/ (Constants.MagEncoderCPR))), new Rotation2d(getTurningRadians())
      );
    
  }
  /**
  * Returns the current angle of the module.
  *
  * @return The current angle of the module in radians.
  */
  public double getTurningRadians() {
      return m_turningMotor.getSelectedSensorPosition() * Constants.turnDistancePerPulse;
  }

  public double getTurnAngle() {
    return Units.radiansToDegrees(getTurningRadians());
  }

  /**
  * Returns the current state of the module.
  *
  * @return The current state of the module.
  */                                                                // test between these two functions
  public SwerveModuleState getState() {                           // to see whther there is a difference between them
   //return new SwerveModuleState(getVelocity(), new Rotation2d(getTurningRadians()));
   return new SwerveModuleState(getVelocity(), new Rotation2d(m_turningMotor.get()));
  }


  public Pose2d getPose() {
    return swerveModulePose;
  }

  public double getMotorVelocity(){
    return getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
