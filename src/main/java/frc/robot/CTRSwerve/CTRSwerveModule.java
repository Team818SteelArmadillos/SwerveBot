package frc.robot.CTRSwerve;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Conversions;

public class CTRSwerveModule {
    private TalonSRX m_driveMotor;
    private TalonSRX m_steerMotor;

    private double m_drivePosition;
    private double m_driveVelocity;
    private double m_steerPosition;
    private double m_steerVelocity;
    // private BaseStatusSignalValue[] m_signals;
    private double m_driveRotationsPerMeter = 0;

    // private PositionVoltage m_angleSetter = new PositionVoltage(0);
    // private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    public CTRSwerveModule(SwerveModuleConstants constants, String canbusName) {
        m_driveMotor = new TalonSRX(constants.DriveMotorId);
        m_steerMotor = new TalonSRX(constants.SteerMotorId);

        // TalonSRXConfiguration talonConfigs = new TalonSRXConfiguration();

        m_driveMotor.configureSlot(constants.DriveMotorGains); //talonConfigs.slot0 = constants.DriveMotorGains;
        m_driveMotor.setNeutralMode(NeutralMode.Brake); // talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // talonConfigs.torqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        // talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        // m_driveMotor.getConfigurator().apply(talonConfigs);
        
        /* Undo changes for torqueCurrent */
        // talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        m_steerMotor.configureSlot(constants.SteerMotorGains); //talonConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        m_steerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); //talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        // talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        // talonConfigs.ClosedLoopGeneral.ContinuousWrap =
        //         true; // Enable continuous wrap for swerve modules

        m_steerMotor.setInverted(constants.SteerMotorReversed);
        // talonConfigs.MotorOutput.Inverted =
        //         constants.SteerMotorReversed
        //                 ? InvertedValue.Clockwise_Positive
        //                 : InvertedValue.CounterClockwise_Positive;
        // m_steerMotor.getConfigurator().apply(talonConfigs);


        //Applying offset here TODO

        // CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        // cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        // m_cancoder.getConfigurator().apply(cancoderConfigs);
        //m_steerMotor.setSelectedSensorPosition(0);
        m_drivePosition = m_driveMotor.getSelectedSensorPosition();
        m_driveVelocity = m_driveMotor.getSelectedSensorVelocity();
        m_steerPosition = m_steerMotor.getSelectedSensorPosition();
        m_steerVelocity = m_steerMotor.getSelectedSensorVelocity();

        // m_signals = new BaseStatusSignalValue[4];
        // m_signals[0] = m_drivePosition;
        // m_signals[1] = m_driveVelocity;
        // m_signals[2] = m_steerPosition;
        // m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    public SwerveModulePosition getPosition() {
        /* Refresh all signals */
        // m_drivePosition.refresh();
        // m_driveVelocity.refresh();
        // m_steerPosition.refresh();
        // m_steerVelocity.refresh();

        m_drivePosition = m_driveMotor.getSelectedSensorPosition();
        m_driveVelocity = m_driveMotor.getSelectedSensorVelocity();
        m_steerPosition = m_steerMotor.getSelectedSensorPosition();
        m_steerVelocity = m_steerMotor.getSelectedSensorVelocity();

        /* Now latency-compensate our signals */
        double drive_rot =
                m_drivePosition; //Ignoring velocity
                        // + (m_driveVelocity * m_drivePosition.laten().getLatency());
        double angle_rot = 
                m_steerPosition;
                        // + (m_steerVelocity.getValue() * m_steerPosition.getTimestamp().getLatency());

        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    public void apply(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(state, m_internalState.angle);

        double angleToSetDeg = optimized.angle.getRotations();
        m_steerMotor.set(ControlMode.Position, Conversions.degreestoTicks(angleToSetDeg, 1) ); //m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        m_driveMotor.set(ControlMode.Velocity, (velocityToSet)); //m_driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
    }

    // BaseStatusSignalValue[] getSignals() {
    //     return m_signals;
    // }

    // public CANcoder getCANCoder() {
    //     return m_cancoder;
    // }
}
