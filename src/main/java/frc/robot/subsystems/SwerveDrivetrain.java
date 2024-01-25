// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrain extends SubsystemBase {

    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveModules;
    public WPI_PigeonIMU m_gyro;

    /**
     * 
     * Constructor for the entire Swerve Drivetrain
     * 
     * Creates all 4 module instances in addition to gyro and odometry configuration
     * 
     */

    public SwerveDrivetrain() {
        m_gyro = new WPI_PigeonIMU(Constants.pigeonID);
        /*m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 11000);
        m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 12000);
        m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 13000);
        m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 14000); */
        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, 
                             Constants.frontLeftRotatePort, 
                             Constants.frontLeftDrivePort, 
                             false,
                             false,
                             Constants.frontLeftOffset),
            new SwerveModule(1,
                             Constants.frontRightRotatePort,
                             Constants.frontRightDrivePort,
                             false,
                             false,
                             Constants.frontRightOffset),
            new SwerveModule(2,
                             Constants.backLeftRotatePort,
                             Constants.backLeftDrivePort,
                             true,
                             false,
                             Constants.backLeftOffset),
            new SwerveModule(3,
                             Constants.backRightRotatePort,
                             Constants.backRightDrivePort,
                             false,
                             false,
                             Constants.backRightOffset)        
                            };
        m_swerveOdometry = new SwerveDriveOdometry(
            Constants.kinematics, 
            getYaw(),
            new SwerveModulePosition[]{
                new SwerveModulePosition(m_swerveModules[0].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[0].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[1].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[1].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[2].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[2].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[3].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[3].getTurnPosition()),
            });
    }

    public SwerveModulePosition[] getPositions(){
        return new SwerveModulePosition[]{
                new SwerveModulePosition(m_swerveModules[0].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[0].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[1].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[1].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[2].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[2].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[3].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[3].getTurnPosition()),
            };
    }
    /**
     * 
     * The drive method for the drivetrain
     * 
     * @param translation Translation2d object containing a vector which represents the distance to be traveled in x and y axes
     * @param rotation The holonomic rotation value from the rotation joystick axis
     * @param fieldRelative If the robot is driving field relative or not, should only be false in the case of a brownout
     * @param isOpenLoop Whether or not the robot is driving using open loop control, almost always false
     * 
     */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        SwerveModuleState[] swerveModuleStates =
            Constants.kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

        SmartDashboard.putNumber("Front Left Turn Encoder", m_swerveModules[0].getRotateTicks() - m_swerveModules[0].getOffset());
        SmartDashboard.putNumber("Front Right Turn Encoder", m_swerveModules[1].getRotateTicks() - m_swerveModules[1].getOffset());
        SmartDashboard.putNumber("Back Left Turn Encoder", m_swerveModules[2].getRotateTicks() - m_swerveModules[2].getOffset());
        SmartDashboard.putNumber("Back Right Turn Encoder", m_swerveModules[3].getRotateTicks()- m_swerveModules[3].getOffset());


        for(var i = 0; i < 4; i++){
            m_swerveModules[i].setDesiredState(swerveModuleStates[i], isOpenLoop, i);
        }


    }

    /**
     * 
     * Sets all 4 modules to the desired states using an array of SwerveModuleStates based on calculations
     * 
     * @param desiredStates An array of all 4 desired states
     * 
     */

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeed);
        
        for(var i = 0; i < 4; i++){
            m_swerveModules[i].setDesiredState(desiredStates[i], false, i);
        }
    }

    /**
     * 
     * Sets all 4 modules to the desired states using a ChassisSpeeds object
     * 
     * @param targetSpeeds The target speeds for all modules in ChassisSpeeds form
     * 
     */
    
    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(Constants.kinematics.toSwerveModuleStates(targetSpeeds));
    }

    /**
     * 
     * @return Pose of the robot in meters 
     * 
     */

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    /**
     * 
     * Sets the robot odometry to the current known pose
     * 
     * @param pose Current pose of the robot
     * 
     */

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), 
        new SwerveModulePosition[]{
                new SwerveModulePosition(m_swerveModules[0].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[0].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[1].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[1].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[2].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[2].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[3].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[3].getTurnPosition()),
            }, 
            pose);
    }

    public void resetOdometryWithYaw(Pose2d pose, Rotation2d degrees) {
        m_swerveOdometry.resetPosition(degrees, 
        new SwerveModulePosition[]{
                new SwerveModulePosition(m_swerveModules[0].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[0].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[1].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[1].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[2].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[2].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[3].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[3].getTurnPosition()),
            },
            pose);
    }

    /**
     * 
     * Gets the theta angle of the robot
     * 
     * @return Current gyro Yaw value in degrees -180-180
     * 
     */

    public double getAngle() {
        double angle = (m_gyro.getYaw());
        return angle;
    }

    public double getNonContinuousGyro() {
        return getAngle() % 360;
    }

    /**
     * 
     * @return Array of SwerveModuleStates containing the states of all 4 robots
     * 
     */

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveModules){
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * 
     * Returns the theta angle of the robot as a Rotation2d object
     * 
     * @return Gyro angle as Rotation2d
     * 
     */

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees((m_gyro.getYaw()));
    }

    /**
     * 
     * Sets the current gyro angle to 0 no matter the robot orientation
     * 
     */

    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * 
     * Sets all module positions to 0 no matter their orientation
     * 
     */

    public void zeroModules() {

        for(SwerveModule mod: m_swerveModules) {
            mod.zeroModule();
        }

    }

    /**
     * 
     * Optimizes the holonomic rotation of the robot
     * 
     * @param currentAngle current theta orientation of the robot
     * @param desiredAngle desired theta orientation of the robot
     * @return which direction the robot should turn: true = left, false = right
     * 
     * Note: WILL ALWAYS RETURN A VALUE, EVEN IF CURRENT = DESIRED SO MAKE SURE TO ONLY RUN THIS METHOD WHEN CURRENT != DESIRED
     * 
     */

    public boolean optimizeTurning(double currentAngle, double desiredAngle) {

        boolean isDesiredPositive = desiredAngle > 0;
        boolean isCurrentPositive = currentAngle > 0;

        double m_desiredAngle = Math.abs(desiredAngle);
        double m_currentAngle = Math.abs(currentAngle);

        double absTotal = Math.abs(currentAngle) + Math.abs(desiredAngle);

        if(isDesiredPositive && isCurrentPositive) {
            if(m_desiredAngle > m_currentAngle) {
                return true;
            } else {
                return false;
            }
        } else if(!isDesiredPositive && !isCurrentPositive) {
            if(m_desiredAngle > m_currentAngle) {
                return true;
            } else {
                return false;
            }
        } else {
            if(absTotal > 180) {
                return false;
            } else {
                return true;
            }
        }
        

    }

    /**
     * 
     * @return array of SwerveModules
     * 
     */

    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    /**
     * 
     * @param yaw yaw value to set the gyro to
     * 
     */

    public void setGyro(double yaw) {
        double yawMod;
        if(yaw < 0) {
            yawMod = 360 - yaw;
        } else if(yaw > 0) {
            yawMod = yaw;
        } else {
            yawMod = yaw;
        }
        m_gyro.setYaw(yawMod);
    }

    /**
     * 
     * Updates odometry with current theta angle and module states
     * 
     * Pushes module cancoder and integrated encoder values, module velocities, and gyro angle to SmartDashboard
     * 
     */

    @Override
    public void periodic(){
        
        m_swerveOdometry.update(getYaw(),
        new SwerveModulePosition[]{
                new SwerveModulePosition(m_swerveModules[0].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[0].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[1].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[1].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[2].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[2].getTurnPosition()),
                new SwerveModulePosition(m_swerveModules[3].getDriveEncoder() * Constants.driveDistancePerPulse, m_swerveModules[3].getTurnPosition()),
            });



        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rot", getPose().getRotation().getDegrees());

    }
}

    