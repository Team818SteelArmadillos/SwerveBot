// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
    private static XboxController testController;
    private static XboxController driveController;
    static{
        driveController = new XboxController(0);
        testController = new XboxController(1);
    }
    public static XboxController getTest(){
        return testController;
    }
    public static XboxController getDrive(){
        return driveController;
    }

}
