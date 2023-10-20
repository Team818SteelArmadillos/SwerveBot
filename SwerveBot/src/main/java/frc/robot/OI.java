package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {

    private static XboxController driveController;
    private static XboxController xopController;
    private static CommandXboxController operatorController;

    //public OI () 
    static {
      operatorController = new CommandXboxController(Constants.OPERATOR_PORT);
      driveController = new XboxController(Constants.DRIVER_PORT);
      xopController = new XboxController(Constants.OPERATOR_PORT);
    }

    public static XboxController getDriver() {
      return driveController;
    }

    public static CommandXboxController getOperator() {
      return operatorController;
    }

    public static XboxController getXOperator() {
      return xopController;
    }
}