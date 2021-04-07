/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics;

import com.chargerrobotics.commands.chomper.ChomperCalibrateCommand;
import com.chargerrobotics.commands.chomper.ChomperDownPIDCommand;
import com.chargerrobotics.commands.chomper.ChomperIntakeCommand;
import com.chargerrobotics.commands.chomper.ChomperUpDownCommand;
import com.chargerrobotics.commands.chomper.ChomperUpPIDCommand;
import com.chargerrobotics.commands.chomper.ChomperVomitCommand;
import com.chargerrobotics.commands.drive.BoostCommand;
import com.chargerrobotics.commands.drive.BrakeCommand;
import com.chargerrobotics.commands.drive.InvertCommand;
import com.chargerrobotics.commands.drive.ManualDriveCommand;
import com.chargerrobotics.commands.drive.SlowCommand;
import com.chargerrobotics.commands.drive.SwitchDriveModeCommand;
import com.chargerrobotics.commands.groups.VisionTurn;
import com.chargerrobotics.commands.shooter.HoodAutoCommand;
import com.chargerrobotics.commands.shooter.HoodCalibrateCommand;
import com.chargerrobotics.commands.shooter.HoodManualCommand;
import com.chargerrobotics.commands.shooter.HoodRetractCommand;
import com.chargerrobotics.commands.shooter.KickerCommand;
import com.chargerrobotics.commands.shooter.ShooterOffCommand;
import com.chargerrobotics.commands.shooter.ShooterOnCommand;
import com.chargerrobotics.subsystems.ChomperSubsystem;
import com.chargerrobotics.subsystems.DriveSubsystem;
import com.chargerrobotics.subsystems.FeedSubsystem;
import com.chargerrobotics.subsystems.KickerSubsystem;
import com.chargerrobotics.subsystems.LimelightSubsystem;
import com.chargerrobotics.subsystems.ShooterHoodSubsystem;
import com.chargerrobotics.subsystems.ShooterSubsystem;
import com.chargerrobotics.utils.Config;
import com.chargerrobotics.utils.XboxController;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static final boolean limelightEnabled = true;
  private static final boolean driveEnabled = true;
  private static final boolean chomperEnabled = true;
  private static final boolean shooterEnabled = true;
  private static final boolean shooterHoodEnabled = true;

  // Limelight
  private LimelightSubsystem limelightSubsystem;

  // Align to the target
  public VisionTurn alignToTarget;

  // Drive
  private DriveSubsystem driveSubsystem;
  private ManualDriveCommand manualDriveCommand;
  private BrakeCommand brakeCommand;
  private BoostCommand boostCommand;
  private SlowCommand slowCommand;
  private InvertCommand invertCommand;
  private SwitchDriveModeCommand switchDriveModeCommand;

  // Shooter
  private ShooterSubsystem shooterSubsystem;
  private KickerSubsystem kickerSubsystem;
  private ShooterHoodSubsystem shooterHoodSubsystem;
  private ShooterOnCommand shooterOnCommand;
  private ShooterOffCommand shooterOffCommand;
  private HoodManualCommand hoodManualUpCommand;
  private HoodManualCommand hoodManualDownCommand;
  private HoodCalibrateCommand hoodCalibrateCommand;
  private HoodAutoCommand hoodAutoCommand;
  private HoodRetractCommand hoodRetractCommand;
  private KickerCommand kickerCommand;

  // Chomper
  private ChomperSubsystem chomperSubsystem;
  private ChomperCalibrateCommand chomperCalibrateCommand;
  private ChomperIntakeCommand chomperIntakeCommand;
  private ChomperVomitCommand chomperVomitCommand;
  private ChomperUpPIDCommand chomperUpCommand;
  private ChomperDownPIDCommand chomperDownCommand;
  private ChomperUpDownCommand manualchomperUpCommand;
  private ChomperUpDownCommand manualchomperDownCommand;

  // Feeder
  private FeedSubsystem feedSubsystem;

  // controllers
  public XboxController primary;
  public XboxController secondary;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    primary = XboxController.getInstance(Constants.primary);
    secondary = XboxController.getInstance(Constants.secondary);
    setupSubsytemsAndCommands();
    setupBindings();
    setupCamera();
  }

  /**
   * Create a container for the robot and initialize subsystems, IO devices, and commands.
   *
   * <p>This overload is typically used for testing purposes. It is recommended to use the
   * constructor with no arguments for most applications.
   *
   * @param primaryController The controller used for the driver system
   * @param secondaryController The controller used for the secondary systems
   */
  public RobotContainer(XboxController primaryController, XboxController secondaryController) {
    primary = primaryController;
    secondary = secondaryController;
    setupSubsytemsAndCommands();
    setupBindings();
    setupCamera();
  }

  private void setupSubsytemsAndCommands() {
    Config.setup();
    if (driveEnabled) {
      driveSubsystem = DriveSubsystem.getInstance(primary);
      manualDriveCommand = new ManualDriveCommand(driveSubsystem, primary);
      brakeCommand = new BrakeCommand(driveSubsystem);
      boostCommand = new BoostCommand(driveSubsystem);
      slowCommand = new SlowCommand(driveSubsystem);
      invertCommand = new InvertCommand(driveSubsystem);
      switchDriveModeCommand = new SwitchDriveModeCommand(driveSubsystem);
    }
    if (limelightEnabled) {
      limelightSubsystem = LimelightSubsystem.getInstance();
      if (driveEnabled) {
        // Vision Testing
        alignToTarget = new VisionTurn(limelightSubsystem, driveSubsystem);
      }
    }
    if (shooterEnabled) {
      shooterSubsystem = ShooterSubsystem.getInstance();
      feedSubsystem = FeedSubsystem.getInstance();
      shooterOnCommand = new ShooterOnCommand(shooterSubsystem);
      shooterOffCommand = new ShooterOffCommand(shooterSubsystem);
      kickerSubsystem = KickerSubsystem.getInstance();
      kickerCommand = new KickerCommand(kickerSubsystem, feedSubsystem);
    }
    if (shooterHoodEnabled) {
      shooterHoodSubsystem = ShooterHoodSubsystem.getInstance();
      hoodManualUpCommand = new HoodManualCommand(shooterHoodSubsystem, true);
      hoodManualDownCommand = new HoodManualCommand(shooterHoodSubsystem, false);
      hoodCalibrateCommand = new HoodCalibrateCommand(shooterHoodSubsystem);
      hoodAutoCommand = new HoodAutoCommand(shooterHoodSubsystem);
      hoodRetractCommand = new HoodRetractCommand(shooterHoodSubsystem);
    }
    if (chomperEnabled) {
      chomperSubsystem = ChomperSubsystem.getInstance();
      feedSubsystem = FeedSubsystem.getInstance();
      chomperCalibrateCommand = new ChomperCalibrateCommand(chomperSubsystem);
      chomperIntakeCommand = new ChomperIntakeCommand(chomperSubsystem, feedSubsystem);
      chomperVomitCommand = new ChomperVomitCommand(chomperSubsystem);
      chomperUpCommand = new ChomperUpPIDCommand(chomperSubsystem);
      chomperDownCommand = new ChomperDownPIDCommand(chomperSubsystem);
      manualchomperUpCommand = new ChomperUpDownCommand(true);
      manualchomperDownCommand = new ChomperUpDownCommand(false);
      CommandScheduler.getInstance()
          .onCommandInitialize(
              (command) -> {
                if (command.equals(chomperUpCommand) && chomperDownCommand.isScheduled()) {
                  chomperDownCommand.cancel();
                } else if (command.equals(chomperDownCommand) && chomperUpCommand.isScheduled()) {
                  chomperUpCommand.cancel();
                }
              });
    }
  }

  public void setupCamera() {
    if (RobotBase.isReal()) {
      // Only setup camera resources when on the real robot as
      // it cannot be simulated easily
      UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
      cam.setConnectVerbose(0);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void setupBindings() {
    // primary
    if (driveEnabled) {
      primary.buttonB.whileHeld(brakeCommand);
      primary.buttonBumperRight.whileHeld(boostCommand);
      primary.buttonBumperLeft.whileHeld(slowCommand);
      primary.buttonStickRight.whenPressed(invertCommand);
      primary.buttonStickLeft.whenPressed(switchDriveModeCommand);
    }
    if (limelightEnabled) {
      primary.buttonX.whileHeld(alignToTarget);
    }
    // secondary
    if (shooterEnabled) {
      secondary.buttonA.whenPressed(shooterOnCommand.alongWith(hoodAutoCommand));
      secondary.buttonB.whenPressed(shooterOffCommand.alongWith(hoodRetractCommand));
      secondary.buttonBumperRight.whileHeld(kickerCommand);
    }
    if (shooterHoodEnabled) {
      secondary.buttonMenu.whenPressed(hoodCalibrateCommand);
      secondary.buttonY.whileHeld(hoodManualUpCommand);
      secondary.buttonX.whileHeld(hoodManualDownCommand);
    }
    if (chomperEnabled) {
      secondary.buttonView.whenPressed(chomperCalibrateCommand);
      secondary.buttonPovUp.whenPressed(chomperUpCommand);
      secondary.buttonPovDown.whenPressed(chomperDownCommand);
      secondary.buttonBumperLeft.whileHeld(chomperIntakeCommand);
      secondary.buttonStickRight.whileHeld(chomperVomitCommand);
      secondary.buttonStickLeft.whileHeld(manualchomperUpCommand);
    }
  }

  public void setDefaultDriveCommand() {
    if (driveEnabled) {
      CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, manualDriveCommand);
    }
  }

  public void setTeleop() {
    if (driveEnabled) {
      manualDriveCommand.schedule();
      if (limelightEnabled) limelightSubsystem.setLEDStatus(false);
    }
  }

  public void setAutonomous() {
    if (driveEnabled && limelightEnabled) {
      limelightSubsystem.setLEDStatus(true);
      alignToTarget.schedule();
    }
  }

  public void setDisabled() {
    if (limelightEnabled) limelightSubsystem.setLEDStatus(false);
  }
}
