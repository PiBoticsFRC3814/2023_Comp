// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.RobotStates;

public class MoveRight extends CommandBase {
  /** Creates a new MoveRight. */
  private GyroSwerveDrive m_drivetrain;
  private RobotStates m_robotStates;
  private Timer timer2;
  private boolean finished;
  public MoveRight(GyroSwerveDrive drivetrain, RobotStates robotStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_robotStates = robotStates;
    timer2 = new Timer();
    addRequirements(drivetrain, robotStates);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer2.reset();
    timer2.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(Constants.SCORE_SPEED, 0, 0);
    if(timer2.hasElapsed(Constants.SCORE_STRAFE_TIME)){
      m_drivetrain.drive(0, 0, 0);
      finished = true;
      m_robotStates.moveFromLastAlign += 1;
      m_robotStates.inFrontOfCubeStation = Math.abs(m_robotStates.moveFromLastAlign) % 3 == 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
