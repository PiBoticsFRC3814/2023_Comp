// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

public class ScoreLow extends CommandBase {
  /** Creates a new ScoreTop. */
  Arm m_Arm;
  Grabber m_Grabber;
  boolean finished;
  public ScoreLow(Arm arm, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Grabber = grabber;
    addRequirements(arm, grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    m_Arm.brake = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.ArmDistance(Constants.EXTEND_REVS_2);
    if(!finished) finished = m_Arm.extendAtPos;
    if(finished) m_Arm.ArmAngle(Constants.SCORE_ANGLE_BOTTOM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
