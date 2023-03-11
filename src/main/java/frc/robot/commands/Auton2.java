// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton2 extends SequentialCommandGroup {
  /** Creates a new Auton1. */
  public Auton2(GyroSwerveDrive drivetrain, RobotStates robotStates, Grabber grabber, Arm arm, ADIS16470_IMU gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GyroReset(gyro),
      new PositionGrid(drivetrain, robotStates),
      new ScoreTop(arm, grabber, robotStates),
      new StowAngle(arm, grabber),
      new AutoDriveDistance(drivetrain, gyro, () -> Constants.AUTON_2_DISTANCE)
    );
  }
}
