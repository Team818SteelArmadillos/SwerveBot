// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.SwerveDrive;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final int kDriverControllerPort = 0;

  public static final int FalconPortFunTime = 0;
  public static final int WheelRadius = 2;
  public static final double DriveMotorGearRatio = 3;
  public static final int pigeonID = 11;
  public static final double maxSpeed = 13; //meters per second
  public static final double maxRotation = 1; //Radians per second
  
  public static final int frontLeftDrivePort = 5;
  public static final int frontRightDrivePort = 4;
  public static final int backLeftDrivePort = 10;
  public static final int backRightDrivePort = 2;
  
  public static final int frontLeftRotatePort = 6;
  public static final int frontRightRotatePort = 7;
  public static final int backLeftRotatePort = 1;
  public static final int backRightRotatePort = 3;

  public static final int MagEncoderCPR = 4096; //NOT SURE ABT THIS

  public static final double driveDistancePerPulse = (WheelRadius * Math.PI) / MagEncoderCPR; //Assumes encoder is direct mounted, probably isn't so fix with gear ration
  public static final double turnDistancePerPulse = (2 * Math.PI) / MagEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kvVoltSecondsPerRadian = 0.8;
    public static final double kaVoltSecondsSquaredPerRadian = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;

    public static final double frameSize = Units.inchesToMeters(26);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(frameSize / 2, frameSize / 2),
      new Translation2d(frameSize / 2, -frameSize / 2),
      new Translation2d(-frameSize / 2, frameSize / 2),
      new Translation2d(-frameSize / 2, -frameSize / 2)
    );
}
