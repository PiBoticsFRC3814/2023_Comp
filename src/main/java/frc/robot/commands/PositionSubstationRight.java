// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotStates;

public class PositionSubstationRight extends CommandBase {

  GyroSwerveDrive m_drivetrain;
  RobotStates m_robotStates;
  Limelight limelight;
  ADIS16470_IMU gyro;

  boolean finished;

  /** Creates a new AutoPosition. */
  public PositionSubstationRight(GyroSwerveDrive drivetrain, RobotStates robotStates, Limelight limelight, ADIS16470_IMU gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_robotStates = robotStates;
    this.limelight = limelight;
    this.gyro = gyro;
    addRequirements(m_drivetrain, m_robotStates, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPositionSetPoint(Constants.RIGHT_SUBSTATION_X, Constants.SUBSTATION_Y);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!finished) {
      m_drivetrain.drive(limelight.outputs);
      finished = limelight.inPosition;
    }
    else m_drivetrain.driveAtHeading(0.0, 0.0, 0.0, gyro.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotStates.inFrontOfCubeStation = true;
    m_robotStates.moveFromLastAlign = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
