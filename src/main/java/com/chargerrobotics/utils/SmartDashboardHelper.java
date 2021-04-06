package com.chargerrobotics.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SmartDashboardHelper {

  public double getNumber(String key, double defaultValue) {
    return SmartDashboard.getNumber(key, defaultValue);
  }

}