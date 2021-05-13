package com.chargerrobotics.subsystems;

import com.chargerrobotics.Constants;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

// useless comment, i have to edit something in the file for a commit to happen

public class ShooterHoodSubsystem extends SubsystemBase {
  private static ShooterHoodSubsystem instance;
  private WPI_TalonSRX shooterHood;
  private boolean calibrated;

  public static ShooterHoodSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterHoodSubsystem();
      CommandScheduler.getInstance().registerSubsystem(instance);
    }
    return instance;
  }

  public ShooterHoodSubsystem() {
    shooterHood = new WPI_TalonSRX(Constants.shooterHood);
    shooterHood.setSafetyEnabled(false);
    shooterHood.configSelectedFeedbackSensor(
        TalonSRXFeedbackDevice.QuadEncoder, Constants.hoodPIDLoopId, Constants.hoodTimeOutMs);
    shooterHood.setNeutralMode(NeutralMode.Brake);
    shooterHood.configPeakCurrentLimit(40);
    shooterHood.configContinuousCurrentLimit(30);
    shooterHood.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }

  public void resetShooterEncoder() {
    shooterHood.setSelectedSensorPosition(0, Constants.hoodPIDLoopId, Constants.hoodTimeOutMs);
  }

  public boolean getCalibrated() {
    return calibrated;
  }

  public void setCalibrated(boolean calibrated) {
    this.calibrated = calibrated;
  }

  public double findHoodTargetTicks(double targetHoodAngDegrees) {
    double outPut = -34.217 * targetHoodAngDegrees + 2532.087;
    return outPut;
  }

  public void setHoodSpeed(double speed) {
    shooterHood.set(speed);
  }

  public double getDesiredAngle() {
    // The distance (inches) reported from the limelight
    double d = SmartDashboard.getNumber("LimelightDistance", 0);
    // Determine the height to the target
    double h = Constants.targetHeight - Constants.cameraHeight;
    // Calculate the angle given height
    double theta = Math.atan(h / d); // was Math.atan(2 * h / d);
    // Return angle in degrees
    return MathUtil.clamp((80 - Math.toDegrees(theta)), 30, 74);
  }

  public boolean isLimitSwitchTriggered() {
    return shooterHood.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public double getHoodPosition() {
    return (double) shooterHood.getSensorCollection().getQuadraturePosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    if (isLimitSwitchTriggered()) resetShooterEncoder();
    SmartDashboard.putNumber("DesiredAngle", getDesiredAngle());
    SmartDashboard.putNumber("DesiredHoodPosition", findHoodTargetTicks(getDesiredAngle()));
    SmartDashboard.putNumber(
        "hoodCurrPos", shooterHood.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("hood Current", shooterHood.getSupplyCurrent());
    SmartDashboard.putBoolean("HoodTriggered?", isLimitSwitchTriggered());
  }
}
