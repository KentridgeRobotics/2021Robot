/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics.commands.shooter;

import com.chargerrobotics.Constants;
import com.chargerrobotics.subsystems.ShooterHoodSubsystem;
import com.chargerrobotics.utils.NetworkMapping;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class HoodPIDCommand extends PIDCommand {
  private static final Logger logger = LoggerFactory.getLogger(HoodPIDCommand.class);
  private ShooterHoodSubsystem shooterHoodSubsystem;
  private static PIDController pid;
  private static double setPoint;

  public static final NetworkMapping<Double> kP =
      new NetworkMapping<Double>(
          "hood_p",
          Constants.hoodP,
          val -> {
            setPIDP(val);
          });

  public static final NetworkMapping<Double> kI =
      new NetworkMapping<Double>(
          "hood_i",
          Constants.hoodI,
          val -> {
            setPIDI(val);
          });

  public static final NetworkMapping<Double> kD =
      new NetworkMapping<Double>(
          "hood_d",
          Constants.hoodD,
          val -> {
            setPIDD(val);
          });

  public static final NetworkMapping<Double> hood_setPoint =
      new NetworkMapping<Double>(
          "hood_SetPointDegrees",
          45.0, // 45.0 is just for testing purposes
          val -> {
            setPoint = val;
          });

  /** Creates a new HoodPIDCommand. */
  public HoodPIDCommand(ShooterHoodSubsystem shooterHoodSubsystem) {
    super(
        // The controller that the command will use
        setPID(new PIDController(kP.getValue(), kI.getValue(), kD.getValue())),
        // This should return the measurement
        () -> shooterHoodSubsystem.getHoodPosition(),
        // This should return the setpoint (can also be a constant)
        () -> shooterHoodSubsystem.findHoodTargetTicks(hood_setPoint.getValue()),
        // This uses the output
        output -> {
          shooterHoodSubsystem.setHoodSpeed(-output);
          SmartDashboard.putNumber("hoodOutput", output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.shooterHoodSubsystem = shooterHoodSubsystem;
    setPIDP(kP.getValue());
    setPIDI(kI.getValue());
    setPIDD(kD.getValue());
    getController().setSetpoint(shooterHoodSubsystem.findHoodTargetTicks(hood_setPoint.getValue()));
    getController().setTolerance(20);
  }

  private static PIDController setPID(PIDController pid) {
    HoodPIDCommand.pid = pid;
    return pid;
  }

  private static void setPIDP(double p) {
    pid.setP(p);
  }

  private static void setPIDI(double i) {
    pid.setI(i);
  }

  private static void setPIDD(double d) {
    pid.setD(d);
  }

  @Override
  public void initialize() {
    if (!shooterHoodSubsystem.getCalibrated()) {
      logger.info("Cannot automatically move hood without calibrating it first!");
      return;
    }
    SmartDashboard.putBoolean("HoodPIDCommandRunning", true);
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putNumber("hoodError", getController().getPositionError());
    SmartDashboard.putNumber("hoodControllerSetpointTicks", getController().getSetpoint());
    SmartDashboard.putNumber("CurrentHoodPosition", shooterHoodSubsystem.getHoodPosition());
    SmartDashboard.putNumber("ControllerP", getController().getP());
    SmartDashboard.putNumber("ControllerI", getController().getI());
    SmartDashboard.putNumber("ControllerD", getController().getD());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooterHoodSubsystem.getCalibrated()) return true;
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    shooterHoodSubsystem.setHoodSpeed(0);
    SmartDashboard.putBoolean("HoodPIDCommandRunning", false);
  }
}
