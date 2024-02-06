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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    }

    public void set(double angleDeg, double driveSpeedOutput)
    {
        double currentAngle = getAngle();
        double angleToSet_delta = (angleDeg % 360) - (currentAngle % 360);
        double speedToSet = driveSpeedOutput;
        
        
        if (isOptimized(angleToSet_delta))
        {
            angleToSet_delta = ((angleDeg + 180) % 360) - (currentAngle % 360);
            speedToSet = -driveSpeedOutput;
        }
        else
        {
           // do nothing
        }
        
        m_azimuthMotor.set(ControlMode.Position, degreesToTicks(currentAngle + angleToSet_delta));
        m_driveMotor.set(ControlMode.PercentOutput, speedToSet);
    }

    private boolean isOptimized(double angleDelta)
    {

        if (angleDelta > 90.0)
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
