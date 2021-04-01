/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.chargerrobotics.subsystems.LimelightSubsystem;
import com.chargerrobotics.subsystems.ShooterHoodSubsystem;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class HoodAutoCommand extends CommandBase {
  private static final Logger logger = LoggerFactory.getLogger(HoodAutoCommand.class);
  private final ShooterHoodSubsystem shooterHoodSubsystem;
  private boolean isOpening;
  /**
   * Creates a new HoodAutoCommand.
   * 
   * Using the distance the robot is from the target, set the hood angle.
   * f(distance)
   */
  public HoodAutoCommand(ShooterHoodSubsystem shooterHoodSubsystem) {
    this.shooterHoodSubsystem = shooterHoodSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //If not calibrated, don't do anything
    if (!shooterHoodSubsystem.getCalibrated()) {
      logger.info("Cannot automatically move hood without calibrating it first!");
      return;
    }
    // Angle to target
    double angle = shooterHoodSubsystem.getDesiredAngleByZone();
    // Ticks to Angle
    double newPosition = shooterHoodSubsystem.findHoodTargetTicks(angle);
    // Current Position
    double position = shooterHoodSubsystem.getHoodPosition();
    // Getting Direction
    isOpening = position < newPosition;
    // Setting Motor Speed
    shooterHoodSubsystem.setHoodSpeed(isOpening ? -0.25 : 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.info("HoodAuto ended");
    // Stopping Motor
    shooterHoodSubsystem.setHoodSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If not calibrated, end the command
    if (!shooterHoodSubsystem.getCalibrated()) return true;
    // Same thing in initialize
    double angle = shooterHoodSubsystem.getDesiredAngleByZone();
    double position = shooterHoodSubsystem.getHoodPosition();
    double newPosition = shooterHoodSubsystem.findHoodTargetTicks(angle);
    // Checking if the Position is Correct
    return isOpening ? position >= newPosition : position <= newPosition;
  }
}