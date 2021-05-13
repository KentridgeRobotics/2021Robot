/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics.commands.chomper;

import com.chargerrobotics.subsystems.ChomperSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ChomperUpPIDCommand extends PIDCommand {
  private static final String name = "ChomperUp";

  private static final Logger logger = LoggerFactory.getLogger(ChomperPIDCommand.class);
  private static final double kP = SmartDashboard.getNumber(name + "P", 0.0004);
  private static final double kI = SmartDashboard.getNumber(name + "I", 0.00014);
  private static final double kD = SmartDashboard.getNumber(name + "D", 0.0);

  /** Creates a new ChomperPIDCommand. */
  public ChomperUpPIDCommand(final ChomperSubsystem chomperSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> (double) chomperSubsystem.chomperUpDownPosition(),
        // This should return the setpoint (can also be a constant)
        () -> chomperSubsystem.getChomperTargetUp(),
        // This uses the output
        output -> {
          chomperSubsystem.setUpDownSpeed(output);
          logger.info("output =" + output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    SmartDashboard.putNumber(name + "P", kP);
    SmartDashboard.putNumber(name + "I", kI);
    SmartDashboard.putNumber(name + "D", kD);
    SmartDashboard.putBoolean("ChomperUpPID running", false);

    this.getController().setTolerance(100);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!ChomperSubsystem.getInstance().isCalibrated()) {
      logger.error("NEED TO CALIBRATE CHOMPER FIRST!!!!");
      return true;
    } else {
      return this.getController().atSetpoint();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    logger.info("Stopping Chomper Up PID Command");
    SmartDashboard.putBoolean("ChomperUpPID running", false);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    this.getController().setP(SmartDashboard.getNumber(name + "P", 0));
    this.getController().setI(SmartDashboard.getNumber(name + "I", 0));
    this.getController().setD(SmartDashboard.getNumber(name + "D", 0));
    logger.info("Started Chomper Up PID Command");
    SmartDashboard.putBoolean("ChomperUpPID running", true);
  }
}
