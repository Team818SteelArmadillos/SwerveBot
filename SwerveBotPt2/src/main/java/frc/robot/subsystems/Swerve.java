// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Swerve extends SubsystemBase{
    SwerveModule[] m_modules;
    Pigeon2 m_pigeon = new Pigeon2(Constants.PigeonID);

    public Swerve(){
        m_modules = new SwerveModule[]{
            new SwerveModule(Constants.frontLeftDrivePort, Constants.frontLeftAzimuthPort, Constants.frontLeftAzimuthOffset, Constants.frontLeftAzimuthInverted),
            new SwerveModule(Constants.frontRightDrivePort, Constants.frontRightAzimuthPort, Constants.frontRightAzimuthOffset, Constants.frontRightAzimuthInverted),
            new SwerveModule(Constants.backLeftDrivePort, Constants.backLeftAzimuthPort, Constants.backLeftAzimuthOffset, Constants.backLeftAzimuthInverted),
            new SwerveModule(Constants.frontRightDrivePort, Constants.frontRightAzimuthPort, Constants.frontRightAzimuthOffset, Constants.frontRightAzimuthInverted),
        };

        for(var i = 0; i < m_modules.length; i++){
            m_modules[i].set(0, 0);
        }
    }

    public Rotation2d getRotation(){
        return m_pigeon.getRotation2d();
    }

    public void drive(double xTranslation, double yTranslation, double rotateSpeed){
        var currentAngle = m_pigeon.getRotation2d();

        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xTranslation, yTranslation, rotateSpeed);

        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentAngle);

        var swerveStates = Constants.m_Kinematics.toSwerveModuleStates(robotSpeeds);

        for(var i = 0; i < m_modules.length; i++){
            m_modules[i].set(swerveStates[i].angle.getDegrees(), swerveStates[i].speedMetersPerSecond);
        }

    }
}
