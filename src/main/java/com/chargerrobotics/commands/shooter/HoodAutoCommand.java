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

public class HoodAutoCommand extends CommandBase {
  private final ShooterHoodSubsystem shooterSubsystem;
  private boolean isOpening;
  /**
   * Creates a new HoodAutoCommand.
   * 
   * Using the distance the robot is from the target, set the hood angle.
   * f(distance)
   */
  public HoodAutoCommand(ShooterHoodSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("hood auto command init");
    // Angle to target
    double angle = shooterSubsystem.getAngleByZone();
    // Ticks to Angle
    double newposition = shooterSubsystem.findHoodTargetTicks(angle);
    // Current Position
    double position = shooterSubsystem.getHoodPosition();
    // Getting Direction
    if (position < newposition) {
       isOpening = true;
    } else {
      isOpening = false;
    }
    // Setting Motor Speed
    shooterSubsystem.setHoodSpeed(isOpening ? -0.25 : 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("hood auto command end");
    // Stopping Motor
    shooterSubsystem.setHoodSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Same thing in initialize
    double angle = shooterSubsystem.getAngleByZone();
    double position = shooterSubsystem.getHoodPosition();
    double newposition = shooterSubsystem.findHoodTargetTicks(angle);
    //System.out.println("Angle: " + angle + ", Position: " + position + ", New position: " + newposition);
    // Checking if the Position is Correct
    return isOpening ? position >= newposition : position <= newposition;
  }
}
