// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.OI;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrive extends Command {

    private double m_rotation;
    private Translation2d m_translation;
    
    private SwerveDrivetrain m_swerveDrivetrain;

    private SlewRateLimiter m_xAxisARateLimiter;
    private SlewRateLimiter m_yAxisARateLimiter;

    /**
     * 
     * Command for driver-controlled driving
     * 
     * @param m_SwerveSubsystem drivetrain instance
     * @param driverController driver XboxController object
     * @param driveAxis corresponding integer for the y-axis translation joystick axis
     * @param strafeAxis corresponding integer for the x-axis translation joystick axis
     * @param rotationAxis corresponding integer for the rotation joystick axis
     * @param fieldRelative whether or not driving is field relative
     * @param openLoop whether or not driving is open loop
     * 
     */

    public SwerveDrive(SwerveDrivetrain m_SwerveSubsystem) {
        m_swerveDrivetrain = m_SwerveSubsystem;
        addRequirements(m_swerveDrivetrain);


        m_xAxisARateLimiter = new SlewRateLimiter(2);
        m_yAxisARateLimiter = new SlewRateLimiter(2);

    }

    @Override
    public void execute() {

        /* Set variables equal to their respective axis */
        double yAxis = -OI.getDrive().getLeftY();
        double xAxis = -OI.getDrive().getLeftX();
        double rAxis = -OI.getDrive().getRightX();
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.DriverDeadzone) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.DriverDeadzone) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.DriverDeadzone) ? 0 : rAxis;

        /* Square joystick inputs */
        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        /* Filter joystick inputs using slew rate limiter */
        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        /* Input variables into drive methods */
        m_translation = new Translation2d(yAxisFiltered, xAxisFiltered).times(Constants.maxSpeed);
        m_rotation = (rAxisSquared * Constants.maxRotation * 0.5);
        m_swerveDrivetrain.drive(m_translation, m_rotation, true, true);

        SmartDashboard.putNumber("Rot", m_rotation);
        SmartDashboard.putNumber("X", m_translation.getX());
        SmartDashboard.putNumber("Y", m_translation.getY());

    }
}