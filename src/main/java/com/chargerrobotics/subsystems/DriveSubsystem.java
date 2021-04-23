package com.chargerrobotics.subsystems;

import com.chargerrobotics.Constants;
import com.chargerrobotics.commands.drive.ManualDriveCommand;
import com.chargerrobotics.utils.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private static DriveSubsystem instance;

  private NetworkTable table;

  private double heading;

  private CANSparkMax leftRear;
  private CANSparkMax leftFront;
  private CANSparkMax rightRear;
  private CANSparkMax rightFront;

  private DifferentialDrive differentialDrive;
  private SpeedControllerGroup leftDriveGroup;
  private SpeedControllerGroup rightDriveGroup;

  private double leftThrottle;
  private double rightThrottle;
  private double arcadeThrottle;
  private double arcadeTurnRate;

  private boolean brake;
  private boolean boost;
  private boolean slow;
  private boolean inverted;
  private boolean inTankDrive;

  private boolean autonomousRunning;

  private XboxController driveController;

  public static DriveSubsystem getInstance(XboxController driveController) {
    if (instance == null) instance = new DriveSubsystem(driveController);
    CommandScheduler.getInstance().registerSubsystem(instance);
    return instance;
  }

  public DriveSubsystem(XboxController driveController) {
    this.driveController = driveController;

    table = NetworkTableInstance.getDefault().getTable("BNO055");

    leftRear = new CANSparkMax(Constants.leftRearDrive, MotorType.kBrushless);
    leftRear.setIdleMode(IdleMode.kCoast);
    leftFront = new CANSparkMax(Constants.leftFrontDrive, MotorType.kBrushless);
    leftFront.setIdleMode(IdleMode.kCoast);
    rightRear = new CANSparkMax(Constants.rightRearDrive, MotorType.kBrushless);
    rightRear.setIdleMode(IdleMode.kCoast);
    rightFront = new CANSparkMax(Constants.rightFrontDrive, MotorType.kBrushless);
    rightFront.setIdleMode(IdleMode.kCoast);
    leftRear.setSmartCurrentLimit(40);
    leftFront.setSmartCurrentLimit(40);
    rightRear.setSmartCurrentLimit(40);
    rightFront.setSmartCurrentLimit(40);

    leftRear.setOpenLoopRampRate(0.4);
    leftFront.setOpenLoopRampRate(0.4);
    rightRear.setOpenLoopRampRate(0.4);
    rightFront.setOpenLoopRampRate(0.4);

    leftDriveGroup = new SpeedControllerGroup(leftFront, leftRear);
    leftDriveGroup.setInverted(true);
    rightDriveGroup = new SpeedControllerGroup(rightFront, rightRear);

    differentialDrive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
    differentialDrive.setDeadband(0.0);

    inverted = false;
    inTankDrive = true;
  }

  public void setAutonomousRunning(boolean autonomousRunning) {
    this.autonomousRunning = autonomousRunning;
    double rampRate = autonomousRunning ? 2.0 : 0.1;
    leftFront.setOpenLoopRampRate(rampRate);
    rightFront.setOpenLoopRampRate(rampRate);
    leftRear.setOpenLoopRampRate(rampRate);
    rightRear.setOpenLoopRampRate(rampRate);
    leftThrottle = rightThrottle = 0;
  }

  public void setThrottle(double left, double right) {
    if (inTankDrive) {
      leftThrottle = left;
      rightThrottle = right;
    } else {
      arcadeThrottle = right; // This is intentional
      arcadeTurnRate = left; // This is also intentional
    }
  }

  public void setBrake(boolean brake) {
    this.brake = brake;
    if (this.brake) {
      leftRear.setIdleMode(IdleMode.kBrake);
      leftFront.setIdleMode(IdleMode.kBrake);
      rightRear.setIdleMode(IdleMode.kBrake);
      rightFront.setIdleMode(IdleMode.kBrake);
    } else {
      leftRear.setIdleMode(IdleMode.kCoast);
      leftFront.setIdleMode(IdleMode.kCoast);
      rightRear.setIdleMode(IdleMode.kCoast);
      rightFront.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getOdoLeft() {
    return leftFront.getEncoder().getPosition();
  }

  public double getOdoRight() {
    return rightFront.getEncoder().getPosition();
  }

  public double getRobotHeading() {
    return heading;
  }

  public boolean getInverted() {
    return inverted;
  }

  public boolean getInTankDrive() {
    return inTankDrive;
  }

  public void setSpeeds(double left, double right) {
    leftDriveGroup.set(left);
    rightDriveGroup.set(right);
  }

  public void setBoost(boolean boost) {
    this.boost = boost;
  }

  public void setSlow(boolean slow) {
    this.slow = slow;
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  public void setInTankDrive(boolean inTankDrive) {
    this.inTankDrive = inTankDrive;
  }

  public void tankDrive(double leftPower, double rightPower) {
    if (this.brake) {
      leftPower *= 0.0;
      rightPower *= 0.0;
    } else if (!this.boost) {
      if (this.slow) {
        leftPower *= 0.3;
        rightPower *= 0.3;
      } else {
        leftPower *= 0.6;
        rightPower *= 0.6;
      }
    }

    if (inverted) {
      differentialDrive.tankDrive(rightPower, leftPower);
      SmartDashboard.putString("DriveMode", "Tank, inverted");
    } else {
      differentialDrive.tankDrive(leftPower, rightPower);
      SmartDashboard.putString("DriveMode", "Tank, Normal");
    }
  }

  public void arcadeDrive(double throttle, double turnRate) {
    if (this.brake) {
      throttle *= 0.0;
      turnRate *= 0.0;
    } else if (!this.boost) {
      if (this.slow) {
        throttle *= 0.3;
        turnRate *= 0.3;
      } else {
        throttle *= 0.6;
        turnRate *= 0.6;
      }
    }
    if (inverted) {
      differentialDrive.arcadeDrive(throttle, -turnRate);
      SmartDashboard.putString("DriveMode", "Arcade, Inverted");
    } else {
      differentialDrive.arcadeDrive(throttle, turnRate);
      SmartDashboard.putString("DriveMode", "Arcade, Normal");
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    heading =
        table
            .getEntry("euler_roll")
            .getDouble(
                -1); // -1 is an invalid value because the BNO055 returns the heading from 0 - 359
    // degrees
    SmartDashboard.putNumber("heading", heading);
    if (!autonomousRunning) {
      if (inTankDrive) {
        tankDrive(leftThrottle, rightThrottle);
      } else {
        arcadeDrive(arcadeThrottle, arcadeTurnRate);
      }
    }
  }

  public void initDefaultCommand() {
    setDefaultCommand(new ManualDriveCommand(this, this.driveController));
  }
}
