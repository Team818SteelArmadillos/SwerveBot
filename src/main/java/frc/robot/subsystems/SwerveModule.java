// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Conversions;
import frc.robot.Constants;
public class SwerveModule {
    public int m_moduleNumber;
    private double m_offset;
    private WPI_TalonSRX m_azimuthMotor;
    private WPI_TalonSRX m_driveMotor;
    private double m_lastAngle;
    private boolean m_turningInverted;
    private boolean m_driveInverted;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);

    /**
     * 
     * The constructor for each swerve module on the robot
     * 
     * @param moduleNumber FL -> 0 FR -> 1 BL -> 2 BR -> 3
     * @param azimuthMotor Azimuth motor CAN ID
     * @param driveMotor Drive motor CAN ID
     * @param azimuthInverted Is the azimuth motor inverted in the forward direction
     * @param driveInverted Is the drive motor inverted in the forward direction
     * 
     */

    public SwerveModule(int moduleNumber, int azimuthMotor, int driveMotor, boolean azimuthInverted, boolean driveInverted, double offset){
        m_moduleNumber = moduleNumber;
        m_turningInverted = azimuthInverted;
        m_driveInverted = driveInverted;
        m_offset = offset;


        m_azimuthMotor = new WPI_TalonSRX(azimuthMotor);
        m_driveMotor = new WPI_TalonSRX(driveMotor);



        configTurningMotor(azimuthMotor);
        configDriveMotor();

        m_lastAngle = Conversions.tickstoDegrees(m_azimuthMotor.getSelectedSensorPosition(), 1);

        m_azimuthMotor.set(ControlMode.Position, m_offset);

        //zeroModule();
    }

    public double getRotateTicks(){
        return m_azimuthMotor.getSelectedSensorPosition();
    }

    public double getOffset(){
        return m_offset;
    }
    /**
     * 
     * Set the state of the individual module to the desired position 
     * 
     * @param desiredState the SwerveModuleState for the individual module to go to
     * @param openLoop should almost always be false, whether driving is open loop or not
     * 
     */

    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop, int moduleNumber) {

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        if(openLoop){
            m_driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / Constants.maxSpeed);
        } else {
            m_driveMotor.set(ControlMode.Velocity, 
                             Conversions.MPStoTicks(desiredState.speedMetersPerSecond, Constants.WheelDiameter * Math.PI, 1), 
                             DemandType.ArbitraryFeedForward, 
                             feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = Math.abs(desiredState.angle.getRadians()) <= (Constants.maxRotation * 0.01) ? m_lastAngle : (desiredState.angle.getDegrees()); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double ticks = Conversions.degreestoTicks(angle, 1);
        SmartDashboard.putNumber("Desired Ticks", ticks);
        m_azimuthMotor.set(ControlMode.Position, ticks);

        m_lastAngle = angle;
    }

    /**
     * 
     * Configures the Azimuth motor to the configuration set in CTREConfigs.java
     * 
     */

    private void configTurningMotor(int azimuthMotor) {
        m_azimuthMotor.configFactoryDefault();
        m_azimuthMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        m_azimuthMotor.setInverted(m_turningInverted);
        m_azimuthMotor.setNeutralMode(NeutralMode.Brake);
        m_azimuthMotor.config_kP(0, 1);
        m_azimuthMotor.config_kD(0, 0);

        
    }

    /**
     * 
     * Configures the Drive motor to the configuration set in CTREConfigs.java
     * 
     */

    private void configDriveMotor() {        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.setInverted(m_driveInverted);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return Rotation2d CANCoder position
     * 
     */

    public Rotation2d getTurnPosition() {
        return Rotation2d.fromDegrees(Conversions.tickstoDegrees(m_azimuthMotor.getSelectedSensorPosition(), Constants.AZIMUTH_GEAR_RATIO));
    }

    /**
     * 
     * @return drive motor integrated encoder position
     * 
     */

    public double getDriveEncoder() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    /**
     * 
     * @return SwerveModuleState state of the individual module
     * 
     */

    public SwerveModuleState getState() {

        double velocity = Conversions.tickstoMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.WheelDiameter * Math.PI, 1);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.tickstoDegrees(m_azimuthMotor.getSelectedSensorPosition(), Constants.AZIMUTH_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);

    }

    /**
     * 
     * Sets the position of the azimuth integrated encoder to the zero position of the CANCoder
     * 
     */

    public void zeroModule() {
        m_azimuthMotor.set(ControlMode.Position, Conversions.degreestoTicks(m_offset, Constants.AZIMUTH_GEAR_RATIO));
    }
}