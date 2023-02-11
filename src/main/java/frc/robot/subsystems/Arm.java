// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonSRX shoulder;
  private WPI_TalonSRX extend;

  public boolean extendAtPos;
  public boolean shoulderAtPos;

  DoubleSolenoid armBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAW_ID_OPEN, Constants.CLAW_ID_CLOSE);

  PIDController angleController;
  boolean switch1;
  boolean switch2;
  boolean switch3;
  boolean switch4;

  public Arm() {
    shoulder = new WPI_TalonSRX(Constants.SHOULDER_ID);
    extend = new WPI_TalonSRX(Constants.EXTEND_ID);

    angleController = new PIDController(0, 0, 0);
    extendAtPos = false;
    shoulderAtPos = false;
  }

  public void ArmAngle(double angle) {
    double encoder = 0.0;

    shoulder.set(angleController.calculate(encoder, angle));
    shoulderAtPos = !angleController.atSetpoint();
    
    if(!shoulderAtPos) 
      armBrake.set(DoubleSolenoid.Value.kReverse);
    else
      armBrake.set(DoubleSolenoid.Value.kForward);
    
  }

  public void ArmDistance(int position) {
    extend.set(-1.0);
    extendAtPos = false;
    while(!extendAtPos) {
      switch(position) {
        case 0:
          extend.set(Constants.EXTEND_SPEED);
          if(switch1){
            extend.set(0.0);
            extendAtPos = true;
          }

        case 1:
          extend.set(Constants.EXTEND_SPEED);
          if(switch2){
            extend.set(0.0);
            extendAtPos = true;
          }
        case 2:
          extend.set(Constants.EXTEND_SPEED);
          if(switch3){
            extend.set(0.0);
            extendAtPos = true;
          }
        case 3:
          extend.set(Constants.EXTEND_SPEED);
          if(switch4){
            extend.set(0.0);
            extendAtPos = true;
          }
      }
    }
  }
}


