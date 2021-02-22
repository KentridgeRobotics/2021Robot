/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

  /**
   * Creates a new AutoTurnStationary.
   * 
   * Turns the robot in place (not moving forwards or backwards). 
   * 
   * Goal is to turn and be able to move in a direction using the gyro and ticks from the encoders
   * get position from gyro and odometry (located in DifferentialDriveOdometry.class) 
   * +- target from current heading 
   * likely will have to use PID for arriving to location                                   
   * 
   */

package com.chargerrobotics.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.chargerrobotics.sensors.GyroscopeSerial;
import com.chargerrobotics.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.chargerrobotics.utils.SmartDashboardHelper;


public class AutoTurnStationary extends CommandBase {
  private static final Logger logger = LoggerFactory.getLogger(AutoTurnStationary.class);

  private double currentHeading;
  private double initialHeading;
  private double targetAngle = 0;     // This is the desired angle
  private double lastRotationOutput; // use this as variable to store value that is being passed through the drive 

  private final DriveSubsystem drive; // look at gyroscopeseial thing for why its not working
  private final GyroscopeSerial gyro; //Will use gyro to check location and angle
  private final SmartDashboardHelper smartDashboardHelper; 

  private DifferentialDriveOdometry odometry; // odometry is like a graph on the floor and directions on how to get back to where it started
  private PIDController rotationPid;  //needed to figure speed for turning 

  public AutoTurnStationary(DriveSubsystem drive, GyroscopeSerial gyro, SmartDashboardHelper smartDashboardHelper) {
    this.drive = drive;
    this.gyro = gyro;
    this.smartDashboardHelper = smartDashboardHelper;
    
    this.rotationPid = new PIDController(0.0, 0.0, 0.0);
    this.rotationPid.enableContinuousInput(0.0, 360.0);
  }

  public void SetTargetAngle(double angle)
  {
    this.targetAngle = angle;
  }

  public PIDController GetRotationPID()
  {
    return this.rotationPid;
  }

  public double GetLastRotationValue()
  {
    return this.lastRotationOutput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.info("AutoTurn starting");

    initialHeading = gyro.getHeading();
    currentHeading = initialHeading;
    drive.setAutonomousRunning(true);
    drive.setThrottle(0, 0); // Clear out any current speed/throttle on the drive....
    
    rotationPid.setP(smartDashboardHelper.getNumber("linRotP", 0.05));
    rotationPid.setI(smartDashboardHelper.getNumber("linRotI", 0.05));
    rotationPid.setD(smartDashboardHelper.getNumber("linRotD", 0.05));
    rotationPid.setTolerance(smartDashboardHelper.getNumber("linRotTol", 0.05));
    rotationPid.setSetpoint(smartDashboardHelper.getNumber("StationaryRotation", targetAngle));
    odometry = new DifferentialDriveOdometry(getGyroHeading(), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

   }
   

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeading = gyro.getHeading();
    Pose2d pose = odometry.update(getGyroHeading(), 0, 0); // getDistanceLeft = 0, getDistanceRight = 0
    Rotation2d rotation = pose.getRotation();
    double rotationOutput = rotationPid.calculate(rotation.getDegrees());
    this.lastRotationOutput = rotationOutput;
    drive.arcadeDrive(0, rotationOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setAutonomousRunning(false);
    logger.info("AutoTurn done");  

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationPid.atSetpoint();
  }


  private Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(currentHeading - initialHeading);
  }
}

