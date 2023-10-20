package frc.robot.subsystems.CTRSwerve;

public class SwerveDriveTrainConstants {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int Pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String CANbusName = "CANivore";

    public double TurnKp = 0;
    public double TurnKi = 0;
    public double TurnKd = 0;

    public double xKp = 0;
    public double xKi = 0;
    public double xKd = 0;
    
    public double yKp = 0;
    public double yKi = 0;
    public double yKd = 0;

    public SwerveDriveTrainConstants withPigeon2Id(int id) {
        this.Pigeon2Id = id;
        return this;
    }

    public SwerveDriveTrainConstants withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKp(double TurnKp) {
        this.TurnKp = TurnKp;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKd(double TurnKd) {
        this.TurnKd = TurnKd;
        return this;
    }
}
