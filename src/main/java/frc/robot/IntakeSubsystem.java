// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


public class IntakeSubsystem extends SubsystemBase { 
  
  private static TalonFX coralIntakeMotor, funnelMotor;;

  public IntakeSubsystem() {

    coralIntakeMotor = new TalonFX(Constants.coralIntakeMotorID, "rio");
    coralIntakeMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive));
    funnelMotor = new TalonFX(Constants.funnelMotorID, "rio");
    funnelMotor.getConfigurator().apply(
      new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive));
    
  }
  public static void setCoralIntakeMotor(double voltage) {
    coralIntakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  public static void setFunnelMotor(double voltage) {
    funnelMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run  }
  }
}
