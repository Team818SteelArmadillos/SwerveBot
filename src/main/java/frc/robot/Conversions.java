// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Conversions {

    /**
     * 
     * Convert falcon counts to degrees
     * 
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     * 
     */

    public static double tickstoDegrees(double counts, double gearRatio) {

        return  360.0 * (counts / (gearRatio * 4096));

    }

    /**
     * 
     * Convert degrees to falcon counts
     * 
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     * 
     */

    public static double degreestoTicks(double degrees, double gearRatio) {

        double ticks =  (degrees / 360.0) * (gearRatio * 4096.0);
        return ticks;

    }

    /**
     * 
     * Convert native falcon units to RPM
     * 
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     * 
     */

    public static double tickstoRPM(double velocityCounts, double gearRatio) {

        double motorRPM = velocityCounts * (600.0 / 4096.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;

    }

    /**
     * 
     * Convert RPM to native falcon units
     * 
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     * 
     */

    public static double RPMtoTicks(double RPM, double gearRatio) {

        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (4096.0 / 600.0);
        return sensorCounts;

    }

    /**
     * 
     * Convert falcon native units to meters per second
     * 
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     * 
     */

    public static double tickstoMPS(double velocitycounts, double circumference, double gearRatio){

        double wheelRPM = tickstoRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;

    }

    /**
     * 
     * Convert Meters per second to falcon native units
     * 
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     * 
     */

    public static double MPStoTicks(double velocity, double circumference, double gearRatio){

        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMtoTicks(wheelRPM, gearRatio);
        return wheelVelocity;

    }

}
