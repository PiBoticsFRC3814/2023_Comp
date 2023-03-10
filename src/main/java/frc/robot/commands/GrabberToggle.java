// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class GrabberToggle extends CommandBase {
  /** Creates a new GrabberCommand. */
  Grabber m_grabber;

  public GrabberToggle(Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_grabber = grabber;
    addRequirements(grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_grabber.clawOpen) m_grabber.GrabberClose();
    else m_grabber.GrabberOpen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
