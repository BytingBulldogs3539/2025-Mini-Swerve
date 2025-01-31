// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
  private static Servo clawServo;
  private static double clawServoPosition = 0;

  public ServoSubsystem() {
    clawServo = new Servo(Constants.clawServoID);

  }

  public static void setClawServoPosition(double position) {
		clawServoPosition = position;
	}
  @Override
  public void periodic() {
    clawServo.set(clawServoPosition);  
    }
  }

