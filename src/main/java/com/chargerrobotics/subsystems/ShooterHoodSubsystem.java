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
    // Get the limelight subsystem
    LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    // The distance (inches) reported from the limelight
    double d = limelight.distance();
    // Determine the height to the target
    double h = Constants.targetHeight - Constants.cameraHeight;
    // Calculate the angle given height
    double theta = Math.atan(2 * h / d);
    // Return angle in degrees
    return Math.toDegrees(theta);
  }

  public double getDesiredAngleByZone() {
    // Get the distance (inches) from the limelight
    LimelightSubsystem limelight = LimelightSubsystem.getInstance();
    double d = limelight.distance();

    // We still need to test the best angles for each zone

    // green zone -- 0 to 90 inches
    if (d >= 0 && d < 90) return 45.0;
    // yellow zone
    if (d >= 90 && d < 150) return 0.0;
    // blue zone
    if (d >= 150 && d < 210) return 0.0;
    // red zone
    if (d >= 210 && d < 270) return 0.0;

    return 0.0;
  }

  public boolean isLimitSwitchTriggered() {
    return shooterHood.getSensorCollection().isFwdLimitSwitchClosed();
    // return !shooterLimitSwitch.get();
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
