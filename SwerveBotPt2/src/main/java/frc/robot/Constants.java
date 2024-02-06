// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int frontLeftDrivePort = 5;
  public static final int frontRightDrivePort = 4;
  public static final int backLeftDrivePort = 1;
  public static final int backRightDrivePort = 2;

  public static final int frontLeftAzimuthPort = 6;
  public static final int frontRightAzimuthPort = 7;
  public static final int backLeftAzimuthPort = 10;
  public static final int backRightAzimuthPort = 3;

  public static final boolean frontLeftAzimuthInverted = false;
  public static final boolean frontRightAzimuthInverted = false;
  public static final boolean backLeftAzimuthInverted = false;
  public static final boolean backRightAzimuthInverted = false;

  public static final double frontLeftAzimuthOffset = 0;
  public static final double frontRightAzimuthOffset = 0;
  public static final double backLeftAzimuthOffset = 0;
  public static final double backRightAzimuthOffset = 0;

  public static final double m_ChasisLength = 26;

  public static final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
  new Translation2d(m_ChasisLength/2, m_ChasisLength/2), 
  new Translation2d(-m_ChasisLength/2, m_ChasisLength/2),
  new Translation2d(m_ChasisLength/2, -m_ChasisLength/2),
  new Translation2d(-m_ChasisLength/2, -m_ChasisLength/2)
  );

  public static final int PigeonID = 11;








}
