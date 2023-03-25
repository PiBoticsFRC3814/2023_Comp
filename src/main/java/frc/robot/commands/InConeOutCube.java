// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.RobotStates;

public class InConeOutCube extends CommandBase {
  /** Creates a new InConeOutCube. */
  Grabber intake;
  Timer intakeTimer;
  RobotStates robotStates;

  double intakeDuration;
  public InConeOutCube(Grabber intake, RobotStates robotStates, double intakeDuration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.robotStates = robotStates;
    this.intakeDuration = intakeDuration;

    intakeTimer = new Timer();

    addRequirements(intake, robotStates);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.reset();
    intakeTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.inConeOutCube();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intakeTimer.get() >= intakeDuration) && robotStates.autonomous;
  }
}
