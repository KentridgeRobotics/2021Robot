/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics.subsystems;

import com.chargerrobotics.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private NetworkTable table;

  private NetworkTableEntry tx, ty, tv, ts, leds, camMode, pip;
  private boolean isRunning;
  private static LimelightSubsystem instance;

  private double x, y, v, s, p;

  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    pip = table.getEntry("getpipe");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    ts = table.getEntry("ts");
    leds = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
  }

  public static LimelightSubsystem getInstance() {
    if (instance == null) {
      instance = new LimelightSubsystem();
      CommandScheduler.getInstance().registerSubsystem(instance);
    }
    return instance;
  }

  public void setLEDStatus(boolean enabled) {
    leds.setDouble(enabled ? 0.0 : 1.0);
    camMode.setNumber(enabled ? 0 : 1);
  }

  // Looking down on the field with the initiation line to the south, the markers
  // would be placed as:
  // 2|                N                  |3
  //
  // W             \(r😏bot)/             E
  //
  //
  // 1| __________________________________|4
  //                   S
  // The lines next to each number represent the way a board covered in
  // retroreflective tape of a specfic color or shape should be placed in respect to
  // each other.
  public double getX() {
    /**
     * If there is no target (v == 0) then return 0.0 for angle... don't want robot to turn to a
     * target that doesn't exist
     */
    if (v == 0) {
      return 0.0;
    } else {
      return x;
    }
  }

  public double getY() {
    return y;
  }

  public double getV() {
    return v;
  }

  public double getS() {
    return s;
  }

  public double distM;
  public double skewangle;
  public double xcoord;
  public double ycoord;
  public double realangle;
  public double compassdeg;

  // cycles through pipelines looking for a target
  public int findPL() {
    int pl = -1;
    for (int b = 0; b <= 10; b++) {
      pip.setNumber(b);
      pl = b;
      if (v == 1) break;
    }
    return pl;
  }

  // finds distance to markers
  public Double distanceM() {
    int pl = findPL();
    if (pl == Constants.shooterTarget || pl == Constants.fuelTarget) return null;
    distM =
        Constants.cameraHeight * (Math.tan(Math.toRadians(Math.abs(y - Constants.cameraAngle))));
    return distM;
  }

  // returns x coordinates in inches
  public Double locX() {
    int pl = findPL();
    double skew;

    if (pl == Constants.MARKER1 || pl == Constants.MARKER3) {
      skew = s * -1.0;
      realangle = skew + x;
    } else if (pl == Constants.MARKER2 || pl == Constants.MARKER4) {
      skew = s + 90.0;
      realangle = skew - x;
    } else {
      return null;
    }
    switch (pl) {
      case 0:
        return null;
      case 1:
        return null;
      case 2:
        xcoord = Constants.MARKER_ONE_X;
        break;
      case 3:
        xcoord = Constants.MARKER_TWO_X;
        break;
      case 4:
        xcoord = Constants.MARKER_THREE_X;
        break;
      case 5:
        xcoord = Constants.MARKER_FOUR_X;
        break;
    }
    Double X = distanceM() * Math.cos(Math.toRadians(realangle)) + xcoord;
    return X;
  }

  // returns y coordinantes in inches
  public Double locY() {

    int pl = findPL();
    double skew;
    if (pl == Constants.MARKER1 || pl == Constants.MARKER3) {
      skew = s * -1.0;
      realangle = skew + x;
    } else if (pl == Constants.MARKER2 || pl == Constants.MARKER4) {
      skew = s + 90.0;
      realangle = skew - x;
    } else {
      return null;
    }
    switch (pl) {
      case 0:
        return null;
      case 1:
        return null;
      case 2:
        ycoord = Constants.MARKER_ONE_Y;
        break;
      case 3:
        ycoord = Constants.MARKER_TWO_Y;
        break;
      case 4:
        ycoord = Constants.MARKER_THREE_Y;
        break;
      case 5:
        ycoord = Constants.MARKER_FOUR_Y;
        break;
    }
    Double Y = distanceM() * Math.sin(Math.toRadians(realangle)) + ycoord;
    return Y;
  }

  // Finds absolute rotation of robot on field,
  public Double degrees() {

    int pl = findPL();
    switch (pl) {
      case 0:
        return null;
      case 1:
        return null;
      case 2:
        compassdeg = 270.00 - Math.abs(s); // marker 1
        break;
      case 3:
        compassdeg = Math.abs(s) + 270.00; // marker 2
        break;
      case 4:
        compassdeg = 90.00 - Math.abs(s); // marker 3
        break;
      case 5:
        compassdeg = Math.abs(s) + 90.00; // marker 4
        break;
    }
    return compassdeg;
  }

  // Distance in inches to shooter target
  public Double distance() {

    int pl = findPL();
    if (pl != 0) {
      return null;
    } else {
      /**
       * Note: Math.tan takes radians...thus the conversion. Finds distance to shooting target
       *
       * <p>Note: Constants must be set precisely to the robots configuration or the distance
       * calculations will be wrong. If all of a sudden the distance is off from one match to
       * another, check the angle of the LimeLight camera. If it gets bumped and the angle changes
       * then everything will be off. That is really the main variable that can get bumped.
       */
      return ((Constants.targetHeight - Constants.cameraHeight)
          / Math.tan(Math.toRadians(Constants.cameraAngle + y)));
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);
    s = ts.getDouble(0.0);
    p = pip.getDouble(0.0);
    Double dist = distance();
    Double distM = distanceM();
    Double lx = locX();
    Double ly = locY();
    Double deg = degrees();
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightS", s);
    SmartDashboard.putNumber("LimelightPipeline", p);

    if (dist != null) {
      SmartDashboard.putNumber("LimelightDistance", dist);
    } else {
      SmartDashboard.putNumber("LimelightDistance", -99999.99);
    }
    if (distM != null) {
      SmartDashboard.putNumber("LimelightDistanceM", distM);
    } else {
      SmartDashboard.putNumber("LimelightDistanceM", -99999.99);
    }
    if (deg != null) {
      SmartDashboard.putNumber("LimelightCardinal", deg);
    } else {
      SmartDashboard.putNumber("LimelightCardinal", -99999.99);
    }
    if (lx != null) {
      SmartDashboard.putNumber("LimelightxCoord", lx);
    } else {
      SmartDashboard.putNumber("LimelightxCoord", -99999.99);
    }
    if (ly != null) {
      SmartDashboard.putNumber("LimelightyCoord", ly);
    } else {
      SmartDashboard.putNumber("LimelightyCoord", -99999.99);
    }
  }
}
