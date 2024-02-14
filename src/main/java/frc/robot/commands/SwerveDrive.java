package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDrive extends Command {

    private double m_rotation;
    private Translation2d m_translation;
    private boolean m_fieldRelative;
    private boolean m_openLoop;

    private Vision m_Vision;
    
    private SwerveDrivetrain m_swerveDrivetrain;
    private XboxController m_driverController;

    private SlewRateLimiter m_xAxisARateLimiter;
    private SlewRateLimiter m_yAxisARateLimiter;

    private PIDController m_rotation_pid;

    private double target_rotation;

    private int pid_delayCounter;

    /**
     * 
     * Command for driver-controlled driving
     * 
     * @param swerveDrivetrain drivetrain instance
     * @param driverController driver XboxController object
     * @param driveAxis corresponding integer for the y-axis translation joystick axis
     * @param strafeAxis corresponding integer for the x-axis translation joystick axis
     * @param rotationAxis corresponding integer for the rotation joystick axis
     * @param fieldRelative whether or not driving is field relative
     * @param openLoop whether or not driving is open loop
     * 
     */

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, XboxController driverController, boolean fieldRelative, boolean openLoop, Vision m_Vision) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        m_driverController = driverController;
        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;
        this.m_Vision = m_Vision;

        m_xAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        m_yAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);

        m_rotation_pid = new PIDController(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);
        m_rotation_pid.setTolerance(Constants.ROTATION_TOLERANCE);

        target_rotation = m_swerveDrivetrain.getAngle();

        pid_delayCounter = 0;
    }

    @Override
    public void execute() {

        /* Set variables equal to their respective axis */
        double yAxis = -m_driverController.getLeftY();
        double xAxis = -m_driverController.getLeftX();
        double rAxis = m_driverController.getRightX();
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.STICK_DEADBAND) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.STICK_DEADBAND) ? 0 : xAxis;
        
        if (Math.abs(rAxis) < Constants.STICK_DEADBAND) {
            if (pid_delayCounter >= 25)
            {
                rAxis = m_rotation_pid.calculate(m_swerveDrivetrain.getAngle(), target_rotation);
            }
            else
            {
                target_rotation = m_swerveDrivetrain.getAngle();
                pid_delayCounter++;
            }
            
        }
        else
        {
            pid_delayCounter = 0;
        } 
        
        SmartDashboard.putNumber("rAxis", rAxis);
        SmartDashboard.putNumber("target_rotation", target_rotation);
        SmartDashboard.putNumber("current_rotation", m_swerveDrivetrain.getAngle());

        /* Square joystick inputs */
        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        /* Filter joystick inputs using slew rate limiter */
        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        /* Input variables into drive methods */
        m_translation = new Translation2d(yAxisFiltered, xAxisFiltered).times(Constants.MAX_SPEED);
        m_rotation =  -rAxisSquared * (Constants.MAX_ANGULAR_VELOCITY * 0.5);
        m_swerveDrivetrain.drive(m_translation, m_rotation, m_fieldRelative, m_openLoop);

        /* Reset our gyro and target angle when we press X */
        if (m_driverController.getXButton())
        {
            m_swerveDrivetrain.resetGyro();
            target_rotation = 0.0;
        }
        m_Vision.updatePoseLimelight();
        m_swerveDrivetrain.resetOdometry(m_Vision.getFusedPose());
    }
}

