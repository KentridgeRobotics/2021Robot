package com.chargerrobotics.utils;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotController;

public class CANSparkMaxSim extends CANSparkMax {

  public double currentSpeed;
  private boolean inverted;

  public CANSparkMaxSim(int deviceID, MotorType type) {
    super(deviceID, type);
    inverted = false;
  }

  @Override
  public void pidWrite(double output) {
    set(output);
  }

  @Override
  public void set(double speed) {
    currentSpeed = inverted ? -speed : speed;
  }

  @Override
  public void setVoltage(double outputVolts) {
    set(outputVolts / RobotController.getBatteryVoltage() * (inverted ? -1 : 1));
  }

  @Override
  public double get() {
    return currentSpeed;
  }

  public double getCurrentSpeed() {
    return currentSpeed;
  }

  @Override
  public void setInverted(boolean isInverted) {
    inverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return inverted;
  }

  @Override
  public void disable() {
    set(0);
  }

  @Override
  public void stopMotor() {
    set(0);
  }
}
