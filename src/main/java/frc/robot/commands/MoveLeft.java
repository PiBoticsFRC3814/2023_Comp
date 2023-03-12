// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.RobotStates;

public class MoveLeft extends CommandBase {
  /** Creates a new MoveLeft. */
  private GyroSwerveDrive drivetrain;
  private RobotStates robotStates;
  private Timer timer1;
  private TrapezoidProfile driveProfile;

  public MoveLeft(GyroSwerveDrive drivetrain, RobotStates robotStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.robotStates = robotStates;
    driveProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(5, 9.8),
       new TrapezoidProfile.State(-Constants.SCORE_STRAFE_DISTANCE, 0.0),
        new TrapezoidProfile.State(0.0, 0.0)
    );

    timer1 = new Timer();

    addRequirements(drivetrain, robotStates);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer1.reset();
    timer1.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = driveProfile.calculate(timer1.get());
    drivetrain.drive(setpoint.velocity, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotStates.moveFromLastAlign -= 1;
    robotStates.inFrontOfCubeStation = robotStates.moveFromLastAlign % 3 == 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
