// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmLevel;
import frc.robot.commands.AutoPosition;
import frc.robot.commands.DirectArmCommand;
import frc.robot.commands.DriveFast;
import frc.robot.commands.DriveSlow;
import frc.robot.commands.GrabberToggle;
import frc.robot.commands.GyroReset;
import frc.robot.commands.MoveLeft;
import frc.robot.commands.MoveRight;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.ScoreMiddle;
import frc.robot.commands.ScoreTop;
import frc.robot.commands.TestExtend;
import frc.robot.commands.drive.GyroSwerveDriveCommand;
import frc.robot.commands.drive.HardBrake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GyroSwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ADIS16470_IMU m_gyrp = new ADIS16470_IMU();

  public final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive();
  public final Arm m_arm = new Arm();
  public final Grabber m_grabber = new Grabber();

  private final AutoPosition m_followAprilTag = new AutoPosition(m_gyroSwerveDrive);
  //private final HardBrake m_brakeAndWait = new HardBrake(m_gyroSwerveDrive);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  Joystick driveStick = new Joystick(2);
  //XboxController driveController = new XboxController(2);
  XboxController armController = new XboxController(1);
  XboxController testController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_gyroSwerveDrive.setDefaultCommand(
        new GyroSwerveDriveCommand(
            () -> driveStick.getX(),
            () -> driveStick.getY(),
            () -> driveStick.getZ(),
            m_gyrp,
            m_gyroSwerveDrive));

    //*
    m_arm.setDefaultCommand(
        new DirectArmCommand(
            m_arm, () -> armController.getRawAxis(3), () -> armController.getLeftY()));
    // */

    m_autoChooser.setDefaultOption("Follow Apriltag", m_followAprilTag);
    //m_autoChooser.addOption("Do Nothing", m_brakeAndWait);
    SmartDashboard.putData(m_autoChooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveStick, 6).whileTrue(new GyroReset(m_gyrp));
    new JoystickButton(driveStick, 5).whileTrue(new HardBrake(m_gyroSwerveDrive));
    new JoystickButton(driveStick, 2).whileTrue(new DriveFast(m_gyroSwerveDrive));
    new JoystickButton(driveStick, 2).whileFalse(new DriveSlow(m_gyroSwerveDrive));

    new JoystickButton(armController, 4).whileTrue(new ScoreTop(m_arm, m_grabber));
    new JoystickButton(armController, 3).whileTrue(new ScoreMiddle(m_arm, m_grabber));
    new JoystickButton(armController, 2).whileTrue(new ScoreLow(m_arm, m_grabber));

    //TODO: Add substation, stow, and deploy
    //new JoystickButton(armController, 1).whileTrue(new ArmSubstation(m_arm, m_grabber));
    //new JoystickButton(armController, 10).whileTrue(new ArmStow(m_arm, m_grabber));
    //new JoystickButton(armController, 9).whileTrue(new ArmDeploy(m_arm, m_grabber));

    new JoystickButton(armController, 5).whileTrue(new MoveLeft(m_gyroSwerveDrive));
    new JoystickButton(armController, 6).whileTrue(new MoveRight(m_gyroSwerveDrive));
    new JoystickButton(armController, 7).whileTrue(new AutoPosition(m_gyroSwerveDrive));
    new JoystickButton(armController, 8).whileTrue(new GrabberToggle(m_grabber));

    new JoystickButton(testController, XboxController.Button.kX.value).whileTrue(new GrabberToggle(m_grabber));
    new JoystickButton(testController, XboxController.Button.kB.value).whileTrue(new ArmLevel(m_arm));
    new JoystickButton(testController, 6).whileTrue(new TestExtend(m_arm, m_grabber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }
}
