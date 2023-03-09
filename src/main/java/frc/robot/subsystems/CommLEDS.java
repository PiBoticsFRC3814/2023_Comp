// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CommLEDS extends SubsystemBase {
  /** Creates a new CommLEDS. */
  private static Relay lightController;
  private static boolean lightsOn = false;
  public CommLEDS() {
    lightController = new Relay(Constants.LIGHT_RELAY_PORT);
  }

  public void lightsCone(){
    lightsOn = false;
  }

  public void lightsCube(){
    lightsOn = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lightController.set(lightsOn ? Relay.Value.kOn : Relay.Value.kOff);
  }
}
