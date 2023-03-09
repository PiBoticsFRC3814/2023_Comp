// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStates extends SubsystemBase {
  /** Creates a new RobotStates. */
  public boolean inFrontOfCubeStation;
  public int moveFromLastAlign;

  public RobotStates() {
    inFrontOfCubeStation = false;
    moveFromLastAlign = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}