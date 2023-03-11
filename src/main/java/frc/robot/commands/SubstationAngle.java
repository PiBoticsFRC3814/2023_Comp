// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

public class SubstationAngle extends CommandBase {
  /** Creates a new ScoreTop. */
  Arm m_Arm;
  Grabber m_Grabber;
  boolean finished1;
  boolean finished2;
  boolean finished3;
  public SubstationAngle(Arm arm, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Grabber = grabber;
    addRequirements(arm, grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished1 = false;
    finished2 = false;
    finished3 = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!finished1) {
      m_Arm.ArmDistance(0);
      finished1 = m_Arm.extendAtPos;
    }

    if(finished1){
      m_Arm.ArmAngle(Constants.SUBSTATION_ANGLE);
      finished2 = m_Arm.shoulderAtPos;
    }

    if(finished1 && finished2 && !finished3){
      m_Arm.ArmDistance(Constants.SUBSTATION_REV);
      finished3 = m_Arm.shoulderAtPos && m_Arm.extendAtPos;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished1 && finished2 && finished3;
  }
}
