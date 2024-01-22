// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FalconSpinnySubsystem extends SubsystemBase {

  private TalonFX falconspinny;
  private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  public FalconSpinnySubsystem() {
    falconspinny = new TalonFX(0);
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    currentLimitsConfigs.SupplyCurrentLimit = 40;
    currentLimitsConfigs.SupplyCurrentThreshold = 80;
    currentLimitsConfigs.SupplyTimeThreshold = 4;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    
    currentLimitsConfigs.StatorCurrentLimit = 40;
    currentLimitsConfigs.StatorCurrentLimitEnable = false;

    talonFXConfiguration.CurrentLimits = currentLimitsConfigs;
    falconspinny.getConfigurator().apply(talonFXConfiguration);
  }

  public void setMotor(double speed){
    if(Math.abs(speed) > 0.05){
      System.out.println(speed);
      falconspinny.set(speed);
    }else{
      falconspinny.set(0);
    }
    SmartDashboard.putNumber("RPM", falconspinny.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Current", falconspinny.getSupplyCurrent().getValueAsDouble());
  }
  
}
