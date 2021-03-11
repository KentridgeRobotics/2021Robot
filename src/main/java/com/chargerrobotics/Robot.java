/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics;

import com.chargerrobotics.subsystems.LEDSubsystem.LEDMode;
import com.chargerrobotics.subsystems.LimelightSubsystem;
import com.chargerrobotics.utils.ArduinoSerialReceiver;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final int TEAM = 3786;

  private RobotContainer robotContainer;

  private boolean isInitialized = false;

  /**
   * Get the {@link RobotContainer} that manages this robot's subsystems and commands, creating it
   * if it doesn't exist
   *
   * @return The robot's {@link RobotContainer}
   */
  public RobotContainer getContainerInstance() {
    if (robotContainer == null) {
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robotContainer = getContainerInstance();
    SmartDashboard.putData(CommandScheduler.getInstance());
    isInitialized = true;
  }

  /**
   * Whether or not the robot has been initialized
   *
   * @return A boolean representing the initialized status
   */
  public boolean isInitialized() {
    return isInitialized;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters operator control mode. */
  @Override
  public void teleopInit() {
    robotContainer.setTeleop();
    ArduinoSerialReceiver.start();
    if (RobotBase.isReal()) {
      robotContainer.leds.setMode(LEDMode.TELEOP);
    }
    LimelightSubsystem.getInstance().setLEDStatus(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once each time the robot enters disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.setDisabled();
    ArduinoSerialReceiver.close();
    if (RobotBase.isReal()) {
      robotContainer.leds.setMode(LEDMode.DISABLED);
    }
  }

  /** This function is called periodically when the robot is disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    ArduinoSerialReceiver.start();
    robotContainer.setAutonomous();
    if (RobotBase.isReal()) {
      robotContainer.leds.setMode(
          LEDMode
              .AUTONOMOUS); // According to RobotContainer, the LEDSubsystem will not initialize in
      // a simulation
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    ArduinoSerialReceiver.close();
    if (RobotBase.isReal()) {
      robotContainer.leds.setMode(
          LEDMode
              .DISABLED); // According to RobotContainer, the LEDSubsystem will not initialize in a
      // simulation
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once each time the robot enters simulation mode. */
  @Override
  public void simulationInit() {
    ArduinoSerialReceiver.close();
  }

  /** This function is called periodically during simulation mode. */
  @Override
  public void simulationPeriodic() {}

  public static ColorWheelColor getTargetColorWheelColor() {
    String data = DriverStation.getInstance().getGameSpecificMessage();
    return data.length() > 0 ? ColorWheelColor.valueOf(data.charAt(0)) : null;
  }

  public static enum ColorWheelColor {
    BLUE('B'),
    GREEN('G'),
    RED('R'),
    YELLOW('Y');

    private final int id;

    private ColorWheelColor(char c) {
      this.id = c;
    }

    public static ColorWheelColor valueOf(char c) {
      return Arrays.stream(values()).filter(color -> color.id == c).findFirst().orElse(null);
    }
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>DO NOT MODIFY
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
