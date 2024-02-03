// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveModule 
{
    private TalonSRX m_driveMotor;
    private TalonSRX m_azimuthMotor;
    private double m_offset; // This value is determined by reading the sensor position when the modules are zeroed to be pointing forward.

    public SwerveModule(int driverMotorPort, int azimuthMotorPort, double offset, boolean isAzimuthMotorInverted)
    {
        m_driveMotor = new TalonSRX(driverMotorPort);
        m_azimuthMotor = new TalonSRX(azimuthMotorPort);

        m_offset = offset;
        optimized = false;

        m_driveMotor.setNeutralMode(NeutralMode.Brake);

        m_azimuthMotor.setInverted(isAzimuthMotorInverted);
        m_azimuthMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void set(double angleDeg, double driveSpeedOutput)
    {
        double angleToSet;
        double speedToSet = driveSpeedOutput;
        
        double currentAngle = getAngle();
        
        if (determineOptimize(angleDeg, currentAngle))
        {
            angleToSet = 
            speedToSet = -driveSpeedOutput;
        }
        else
        {
            angleToSet = angleDeg;
            speedToSet = driveSpeedOutput;
        }

        angleToSet = angleDeg;
        speedToSet = driveSpeedOutput;

        m_azimuthMotor.set(ControlMode.Position, angleToSet);
        m_driveMotor.set(ControlMode.PercentOutput, speedToSet);
    }

    private boolean determineOptimize(double targetAngle, double currentAngle)
    {
        double angleDelta = Math.abs(currentAngle - targetAngle);

        if (angleDelta > 90)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public double getAngle()
    {
        return ticksToDegrees(m_azimuthMotor.getSelectedSensorPosition() - m_offset);
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
