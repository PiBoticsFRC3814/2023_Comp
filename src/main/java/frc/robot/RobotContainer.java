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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ADIS16470_IMU m_gyrp = new ADIS16470_IMU();

  public final Arm m_arm = new Arm();
  public final Grabber m_grabber = new Grabber();
  public final RobotStates m_robotStates = new RobotStates();
  public final GyroSwerveDrive m_gyroSwerveDrive = new GyroSwerveDrive(m_robotStates);

  private final CommandBase m_brakeAndWait = new HardBrake(m_gyroSwerveDrive);
  private final CommandBase m_auton1 = new Auton1(m_gyroSwerveDrive, m_robotStates, m_grabber, m_arm, m_gyrp);
  private final CommandBase m_auton2 = new Auton2(m_gyroSwerveDrive, m_robotStates, m_grabber, m_arm, m_gyrp);
  private final CommandBase m_balanceAuton = new AutonWithBalance(m_gyroSwerveDrive, m_robotStates, m_grabber, m_arm, m_gyrp);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  Joystick driveStick = new Joystick(2);
  XboxController armController = new XboxController(1);
  //XboxController testController = new XboxController(0);

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

    m_autoChooser.setDefaultOption("Do Nothing", m_brakeAndWait);
    m_autoChooser.addOption("Far left", m_auton1);
    m_autoChooser.addOption("Far Right", m_auton2);
    m_autoChooser.addOption("Center with balance (WIP)", m_balanceAuton);
    SmartDashboard.putData("Auton Chooser", m_autoChooser);

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
    new JoystickButton(driveStick, 4).whileTrue(new PositionSubstation(m_gyroSwerveDrive, m_gyrp));
    //new JoystickButton(driveStick, 3).whileTrue(new PositionSubstationFix(m_gyroSwerveDrive, m_gyrp));
    new JoystickButton(driveStick, 1).whileTrue(new LightsCube());
    new JoystickButton(driveStick, 3).whileTrue(new AutoBalance(m_gyroSwerveDrive, m_gyrp));

    new JoystickButton(armController, 4).whileTrue(new ScoreTop(m_arm, m_grabber, m_robotStates));
    new JoystickButton(armController, 3).whileTrue(new ScoreMiddle(m_arm, m_grabber, m_robotStates));
    new JoystickButton(armController, 2).whileTrue(new ScoreLow(m_arm, m_grabber));

    new JoystickButton(armController, 1).whileTrue(new SubstationAngle(m_arm, m_grabber));
    new JoystickButton(armController, 5).whileTrue(new SubstationAngle(m_arm, m_grabber));
    new JoystickButton(armController, 9).whileTrue(new SubstationAngle(m_arm, m_grabber));
    new JoystickButton(armController, 10).whileTrue(new SubstationOverride(m_arm));
    new Trigger(() -> armController.getPOV(0) == 0).whileTrue(new SubstationAngle(m_arm, m_grabber));
    new Trigger(() -> armController.getPOV(0) == 90).whileTrue(new SubstationAngle(m_arm, m_grabber));
    new Trigger(() -> armController.getPOV(0) == 180).whileTrue(new SubstationAngle(m_arm, m_grabber));
    new Trigger(() -> armController.getPOV(0) == 270).whileTrue(new SubstationAngle(m_arm, m_grabber));
    //new JoystickButton(armController, 10).whileTrue(new ArmStow(m_arm, m_grabber));
    new JoystickButton(armController, 6).whileTrue(new DeployAngle(m_arm, m_grabber));

    //new JoystickButton(armController, 5).whileTrue(new MoveLeft(m_gyroSwerveDrive, m_robotStates));
    //new JoystickButton(armController, 6).whileTrue(new MoveRight(m_gyroSwerveDrive, m_robotStates));
    new JoystickButton(armController, 7).whileTrue(new PositionGrid(m_gyroSwerveDrive, m_robotStates, m_gyrp, m_grabber));
    new JoystickButton(armController, 8).whileTrue(new GrabberToggle(m_grabber, m_robotStates));

    /*
    new JoystickButton(testController, XboxController.Button.kX.value).whileTrue(new GrabberToggle(m_grabber, m_robotStates));
    new JoystickButton(testController, XboxController.Button.kB.value).whileTrue(new ArmLevel(m_arm));
    new JoystickButton(testController, 6).whileTrue(new TestExtend(m_arm, m_grabber)); */
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
