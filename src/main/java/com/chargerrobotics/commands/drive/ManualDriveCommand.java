package com.chargerrobotics.commands.drive;

import com.chargerrobotics.subsystems.DriveSubsystem;
import com.chargerrobotics.utils.XboxController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualDriveCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private XboxController primary;

  public ManualDriveCommand(DriveSubsystem driveSubsystem, XboxController driverController) {
    this.driveSubsystem = driveSubsystem;
    this.primary = driverController;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(driveSubsystem.getInTankDrive()) {
      driveSubsystem.setThrottle(primary.getLeftStickY(), -primary.getRightStickY());
    } else {
      driveSubsystem.setThrottle(primary.getLeftStickY(), -primary.getRightStickX());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
