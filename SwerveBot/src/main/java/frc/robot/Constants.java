// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int OPERATOR_PORT = 2;
  public static final int DRIVER_PORT = 0;

  public static final int PIGEON_ID = 12;
  public static final String CAN_BUS_DRIVE = "CANivore";

  public static final double DRIVE_TURN_KP = 5.0;
  public static final double DRIVE_TURN_KD = 0.4;

  public static final double DRIVE_GEAR_RATIO = (6.55/1.0);
  public static final double AZIMUTH_GEAR_RATIO = (10.29/1.0);

  public static final double WHEEL_RADIUS_INCHES = 2;

  public static final int SLIP_CURRENT = 50;

  /* CAN IDs */
  public static final int BACK_LEFT_AZIMUTH = 6 ;
  public static final int BACK_LEFT_DRIVE = 7;
  public static final int BACK_LEFT_CANCODER = 11;
  public static final double BACK_LEFT_OFFSET = -0.097412;

  public static final int BACK_RIGHT_AZIMUTH = 5;
  public static final int BACK_RIGHT_DRIVE = 4;
  public static final int BACK_RIGHT_CANCODER = 10;
  public static final double BACK_RIGHT_OFFSET = -0.769287;

  public static final int FRONT_RIGHT_AZIMUTH = 3;
  public static final int FRONT_RIGHT_DRIVE = 2;
  public static final int FRONT_RIGHT_CANCODER = 9;
  public static final double FRONT_RIGHT_OFFSET = -0.522705;

  public static final int FRONT_LEFT_AZIMUTH = 0;
  public static final int FRONT_LEFT_DRIVE = 1;
  public static final int FRONT_LEFT_CANCODER = 8;
  public static final double FRONT_LEFT_OFFSET = -0.212646;

  public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(26);

  public static final double STEER_GAINS_KP = 30;
  public static final double STEER_GAINS_KD  = 0.2;
  public static final double DRIVE_GAINS_KP = 1;



}
