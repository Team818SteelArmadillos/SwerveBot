// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule extends SubsystemBase
{
    private TalonSRX m_driveMotor;
    private TalonSRX m_azimuthMotor;
    private double m_offset; // This value is determined by reading the sensor position when the modules are zeroed to be pointing forward.

    public SwerveModule(int driverMotorPort, int azimuthMotorPort, double offset, boolean isAzimuthMotorInverted)
    {
        m_driveMotor = new TalonSRX(driverMotorPort);
        m_azimuthMotor = new TalonSRX(azimuthMotorPort);

        m_offset = offset;

        m_driveMotor.setNeutralMode(NeutralMode.Brake);

        m_azimuthMotor.setInverted(isAzimuthMotorInverted);
        m_azimuthMotor.setNeutralMode(NeutralMode.Brake);

        m_azimuthMotor.config_kP(0, 0.1);

        m_azimuthMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    public void set(SwerveModuleState m_desiredState)
    {
        // double angleDeg = m_desiredState.angle.getDegrees();
        // double currentAngle = getAngle().getDegrees();
        // double angleToSet_delta = (angleDeg % 360) - (currentAngle % 360);
        // double speedToSet = m_desiredState.speedMetersPerSecond;
        
        
        // if (isOptimized(angleToSet_delta))
        // {
        //     angleToSet_delta = ((angleDeg + 180) % 360) - (currentAngle % 360);
        //     speedToSet = -speedToSet;
        // }
        // else
        // {
        //    // do nothing
        // }

        // var desiredState = m_desiredState;

        var desiredStateOptimized = SwerveModuleState.optimize(m_desiredState, new Rotation2d(Constants.ticksToDegrees(m_azimuthMotor.getSelectedSensorPosition())));
        // var desiredStateOptimized = optimizeState(m_desiredState);
        
        m_azimuthMotor.set(ControlMode.Position, Constants.rotationToTicks(desiredStateOptimized.angle));
        m_driveMotor.set(ControlMode.PercentOutput, m_desiredState.speedMetersPerSecond / Constants.maxSpeed);
    }

    private boolean isOptimized(double angleDelta)
    {

        if (Math.abs(angleDelta) > 90.0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private SwerveModuleState optimizeState(SwerveModuleState original){
        double position = getAngle().getDegrees();
        position %= 360;

        double setpoint = original.angle.getDegrees();

        double forward = setpoint + 360;
        double reverse = setpoint - 360;
        double antisetPoint = reverseAngle(setpoint);
        double antiforward = antisetPoint + 360;
        double antireverse = antisetPoint - (360);

        double[] alternatives = {forward, reverse, antisetPoint, antiforward, antireverse};
        double min = setpoint;
        double minDistance = getDistance(setpoint, position);
        int minIndex = -1;
        for(int i = 0; i < alternatives.length; i++){
            double dist = getDistance(alternatives[i], position);
            if(dist == minDistance){
                min = alternatives[i];
                minDistance = dist;
                minIndex = i;
            }
        }
        double speed = original.speedMetersPerSecond;
        if (minIndex > 1){
            speed *= -1;
        }

        Rotation2d myrotation = Rotation2d.fromDegrees(min);
        
        return new SwerveModuleState(speed, myrotation);
    }

    private double getDistance(double setpoint, double position){
        return Math.abs(setpoint - position);
    }

    private double reverseAngle(double degrees){
        return 360 - degrees;
    }

    public Rotation2d getAngle()
    {
        return new Rotation2d(ticksToDegrees(m_azimuthMotor.getSelectedSensorPosition() - m_offset));
    }

    public double ticksToDegrees(double ticks)
    {
        return  360.0 * (ticks / 4096.0);
    }

    public double degreesToTicks(double degrees)
    {
        return   4096.0 * (degrees / 360.0);
    }

}
