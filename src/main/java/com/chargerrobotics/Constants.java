/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Controller IDs
  public static final int primary = 0;
  public static final int secondary = 1;

  // CAN Id's
  // 1: Power distribution panel (PDP)
  // 2: Pneumatic Control Module (PCM)
  // 1x: Drive: Odds are left, evens are right
  // 2x: Shooter
  // 3x: Color Spinner
  public static final int powerDistributionPanel = 1;
  public static final int rightFrontDrive = 11;
  public static final int rightRearDrive = 12;
  public static final int leftRearDrive = 13;
  public static final int leftFrontDrive = 14;
  public static final int shooterLeft = 21;
  public static final int shooterRight = 22;
  public static final int shooterHood = 23;
  public static final int colorSpinner = 31;
  public static final int chomperFeed = 41;
  public static final int chomperLift = 42;
  public static final int feedStage = 43;
  public static final int kicker = 44;
  public static final int climbExtender = 51;
  public static final int climbPush2 = 52;
  public static final int climbPull = 53;

  // PWM IDs
  public static final int colorSpinnerLifter = 9;
  public static final int leds = 1;

  // Digital In/Out Ports
  public static final int chomperLimitSwitch = 9;
  public static final int hoodLimitSwitch = 1;

  // Chomper Constants
  public static final int chomperDistToDown = 550;
  public static final int chomperDistBottomToUp = 1999;

  // Shooter Constants
  public static final double shooterTargetRPM = 2500.0;
  public static final double shooterP = 0.0005;
  public static final double shooterI = 0.00000047;
  public static final double shooterD = 0.014;
  public static final int shooterCurrentLimit = 40;
  public static final double shooterFeedForward = 0.0;
  public static final double shooterStaticGain = 0.0;
  public static final double shooterVelocityGain = 0.0;
  public static final double shooterMinOutput = -1.0;
  public static final double shooterMaxOutput = 1.0;

  public static final double desiredDistance = 120.0;

  // Shooter Hood Constants
  public static final int hoodPIDLoopId = 0;
  public static final int hoodGainSlot = 0;
  public static final int hoodTimeOutMs = 30;
  public static final int ticksPerRev = 0;
  public static final double hoodP = 0.0007;
  public static final double hoodI = 0.00038;
  public static final double hoodD = 0.0;
  public static final double hoodF = 0.0;

  public static final double hoodPresetAngle = 500;
  public static final double hoodRetractAngle = 1800;
  public static final double defaultHoodSpeed = 0.30;

  public static final class DriveMetrics {
    public static final double DRIVE_GEAR_RATIO =
        1.0; // My measurements bypass the gear reductions. If needed, it's 7.31 to 1.
    public static final double WHEEL_RADIUS_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double TRACK_WIDTH_METERS = 0.65;
    public static final double ENCODER_CPR =
        42.0; // Counts per revolution according to NEO motor datasheet
    public static final double ENCODER_PPR =
        ENCODER_CPR / 4; // Pulses per revolution; 1 count is 4 pulses
    public static final double MOTOR_ROTATIONS_PER_WHEEL_REV =
        12.1425; // average of 13.732 (right) and 10.553 (left)
    public static final double DISTANCE_PER_PULSE =
        WHEEL_CIRCUMFERENCE_METERS / (ENCODER_PPR * MOTOR_ROTATIONS_PER_WHEEL_REV);

    public static final double MAX_VELOCITY = 3.0; // Meters per second
    public static final double MAX_ACCEL = 3.0; // Metes squared per second

    // The following values will be updated after running the characterization tool on the robot
    public static final double KS_VOLTS = 0.22; // Volts to overcome static friction
    public static final double KV_LINEAR = 1.98; // Volt * seconds per meter
    public static final double KA_LINEAR = 0.2; // Volt * seconds squared per meter
    public static final double KV_ANGULAR = 1.5; // Volt * seconds per radian - for simulation
    public static final double KA_ANGULAR =
        0.3; // Volt * seconds squared per radian - for simulation
    public static final double KP = 0.9; // P value for the trajectory-following PID controllers
    public static final int MAX_DRIVE_VOLTS = 7;

    // According to documentation, the following values produce desirable results
    public static final double RAMSETE_B =
        2.0; // Changes how aggressively the velocites are adjusted
    public static final double RAMSETE_ZETA = 0.7; // Damping value for velocities
  }

  // File Names
  public static final String dataStoragePath = "/home/lvuser";
  public static final String configFileName = "config.yml";

  // Limelight distance calculation constants
  // Blitzen config:
  // public static final double targetHeight = 94.0; // inches
  // public static final double cameraHeight = 24.0; // inches
  // public static final double cameraAngle = 30.0; // degrees

  // 2020 bot config:
  public static final double targetHeight = 91.5; // inches
  public static final double cameraHeight = 23.75; // inches
  public static final double cameraAngle = 29.0; // degrees
  public static final String comPortsFileName = "com.yml";

  /*
  Tests at Tahoma
  Front bumper on the line: 3000 RPM, 55 degrees hood, distance 131 inches
  */
}
