// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  private DoubleSolenoid pivot = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAW_ID_OPEN, Constants.CLAW_ID_CLOSE);
  private WPI_TalonSRX intake;
  private RobotStates robotStates;
  
  /** Creates a new Grabber. */
  public Grabber(RobotStates robotStates) {
    intake = new WPI_TalonSRX(Constants.INTAKE_ID);
    this.robotStates = robotStates;
  }

  public void inConeOutCube(){
    intake.set(-1.0);
  }

  public void inCubeOutCone(){
    intake.set(1.0);
  }

  public void stop(){
    intake.set(0.0);
  }

  private void tiltUp(){
    pivot.set(DoubleSolenoid.Value.kReverse);
  }

  private void tiltDown(){
    pivot.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    if(robotStates.angleArm <= 0.4){
      tiltUp();
    } else tiltDown();
    // This method will be called once per scheduler run
  }
}
