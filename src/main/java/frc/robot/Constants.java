package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public final class Constants {

    /*============================
               Numbers 
    ==============================*/

    public static final double TICKS_PER_REVOLUTION = 4096.0;
    public static final double DEGREES_PER_RADIAN = 57.2958;

    /*============================
               Swerve 
    ==============================*/

    /* CAN IDs */
    public static final int BACK_LEFT_DRIVE = 1; //Josh
    public static final int BACK_LEFT_AZIMUTH = 10; //Tracy

    public static final int BACK_RIGHT_DRIVE = 2; //Happy
    public static final int BACK_RIGHT_AZIMUTH = 3; //Samuel

    public static final int FRONT_RIGHT_DRIVE = 4; //Keith
    public static final int FRONT_RIGHT_AZIMUTH = 7; //Beth

    public static final int FRONT_LEFT_DRIVE = 5; //Chad
    public static final int FRONT_LEFT_AZIMUTH = 6; //Geraldine

    public static final int PIGEON = 11;

    /* Azimuth Encoder offsets */
    public static double FRONT_LEFT_OFFSET  = 0.0;
    public static double FRONT_RIGHT_OFFSET = 0.0;
    public static double BACK_LEFT_OFFSET   = 0.0;
    public static double BACK_RIGHT_OFFSET  = 0.0;

    /* Azimuth reversed */
    public static boolean FRONT_LEFT_AZIMUTH_REVERSED = false;
    public static boolean FRONT_RIGHT_AZIMUTH_REVERSED = false;
    public static boolean BACK_LEFT_AZIMUTH_REVERSED = false;
    public static boolean BACK_RIGHT_AZIMUTH_REVERSED = false;

    /* Drive motors reversed */
    public static boolean FRONT_LEFT_DRIVE_REVERSED = false;
    public static boolean FRONT_RIGHT_DRIVE_REVERSED = false;
    public static boolean BACK_LEFT_DRIVE_REVERSED = false;
    public static boolean BACK_RIGHT_DRIVE_REVERSED = false;

    /* Azimuth Encoder reversed */
    public static boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static boolean BACK_LEFT_CANCODER_REVERSED = false;
    public static boolean BACK_RIGHT_CANCODER_REVERSED = false;

    /* Gyro reversed */
    public static final boolean INVERT_GYRO = false;

    /* Angle Motor PID Values */
    public static final double AZIMUTH_P = 1.0;
    public static final double AZIMUTH_I = 0.0;
    public static final double AZIMUTH_D = 0.1;
    public static final double AZIMUTH_F = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_P = 0.0; 
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_F = 0.0;

    
    /* Robot Rotation PID Values */
    public static final double ROTATION_P = 1.0; 
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double ROTATION_TOLERANCE = 0.0;

    /* Drive Motor Characterization Values */
    public static final double DRIVE_S = (0.48665 / 12); //Values from SysId divided by 12 to convert to volts for CTRE
    public static final double DRIVE_V = (2.4132 / 12);
    public static final double DRIVE_A = (0.06921 / 12);

    /* Azimuth Current Limiting */
    public static final int AZIMUTH_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int AZIMUTH_PEAK_CURRENT_LIMIT = 40;
    public static final double AZIMUTH_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

    /* Drive Current Limiting */
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* Neutral Modes */
    public static final NeutralMode AZIMUTH_NEUTRAL_MODE = NeutralMode.Coast;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

    /* Swerve Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = (1.0); //Ratio from Encoder to Wheel
    public static final double AZIMUTH_GEAR_RATIO = (1.0); //Ratio from Encoder to Azimuth

    /* Swerve Profiling Values */
    public static final double MAX_SPEED = (Units.feetToMeters(16.2)); //meters per second (theoretical from SDS)
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12; //radians per second (theoretical calculation)
    public static final double TURN_IN_PLACE_SPEED = 0.5;
    public static final double A_RATE_LIMITER = 2.0; //Slew Rate Limiter Constant
    public static final double CONTROLLER_ROTAION_RATE = MAX_ANGULAR_VELOCITY/50.0; //updates per execute cycle AKA every 20ms

    /*============================
               Kinematics
    ==============================*/

    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(19.5); // from center on module to center of module
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(19.5); // from center on module to center of module
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);

    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));

    /*============================
         Controller Constants
    ==============================*/

    /* Controller Constants */
    public static final double STICK_DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int TEST_PORT = 5;
    public static final double OP_RUMBLE_PERCENT = 0.4;
    public static final double DRIVER_RUMBLE_PERCENT = 0.4;
    public static final RumbleType DRIVER_RUMBLE_LEFT = RumbleType.kLeftRumble;
    public static final RumbleType OP_RUMBLE_LEFT = RumbleType.kLeftRumble;
    public static final RumbleType DRIVER_RUMBLE_RIGHT = RumbleType.kRightRumble;
    public static final RumbleType OP_RUMBLE_RIGHT = RumbleType.kRightRumble;
    public static final double DRIVING_INTAKE_RUMBLE = 0.3;

} 
