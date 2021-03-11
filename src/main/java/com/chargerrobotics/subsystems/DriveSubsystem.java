package com.chargerrobotics.subsystems;

import com.chargerrobotics.Constants;
import com.chargerrobotics.commands.drive.ManualDriveCommand;
import com.chargerrobotics.utils.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private static DriveSubsystem instance;

  private CANSparkMax leftRear;
  private CANSparkMax leftFront;
  private CANSparkMax rightRear;
  private CANSparkMax rightFront;

  private edu.wpi.first.wpilibj.Encoder fakeLeftEncoder;
  private edu.wpi.first.wpilibj.Encoder fakeRightEncoder;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private AnalogGyro fakeGyro;
  private AnalogGyroSim gyroSim;

  private Field2d field;

  private DifferentialDrivetrainSim drivetrainSim;

  private DifferentialDrive differentialDrive;
  private SpeedControllerGroup leftDriveGroup;
  private SpeedControllerGroup rightDriveGroup;

  private DifferentialDriveOdometry odometry;

  private double leftThrottle;
  private double rightThrottle;

  private boolean brake;
  private boolean boost;
  private boolean slow;

  private boolean autonomousRunning;
  private boolean odometryInitialized;

  private XboxController driveController;

  private float gyroHeadingRadians;

  public static DriveSubsystem getInstance(XboxController driveController) {
    if (instance == null) instance = new DriveSubsystem(driveController);
    CommandScheduler.getInstance().registerSubsystem(instance);
    return instance;
  }

  public DriveSubsystem(XboxController driveController) {
    this.driveController = driveController;

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

    odometryInitialized = false;

    resetOdometry();

    if (RobotBase.isSimulation()) {
      fakeLeftEncoder = new Encoder(1, 2);
      fakeLeftEncoder.setDistancePerPulse(Constants.DriveMetrics.DISTANCE_PER_PULSE);
      fakeRightEncoder = new Encoder(3, 4);
      fakeRightEncoder.setDistancePerPulse(Constants.DriveMetrics.DISTANCE_PER_PULSE);
      leftEncoderSim = new EncoderSim(fakeLeftEncoder);
      rightEncoderSim = new EncoderSim(fakeRightEncoder);
      fakeGyro = new AnalogGyro(1);
      gyroSim = new AnalogGyroSim(fakeGyro);

      drivetrainSim =
          new DifferentialDrivetrainSim(
              DCMotor.getNEO(2),
              Constants.DriveMetrics.DRIVE_GEAR_RATIO,
              Constants.DriveMetrics.MOMENT_OF_INERTIA,
              Constants.DriveMetrics.ROBOT_MASS_KG,
              Constants.DriveMetrics.WHEEL_RADIUS_METERS,
              Constants.DriveMetrics.TRACK_WIDTH_METERS,
              null); // Will not account for sensor measurement noise.
      field = new Field2d();
      SmartDashboard.putData("Field", field);
    }
  }

  public void initOdometry(double x, double y) {
    odometry =
        new DifferentialDriveOdometry(
            RobotBase.isReal()
                ? new Rotation2d(gyroHeadingRadians)
                : Rotation2d.fromDegrees(gyroSim.getAngle()),
            new Pose2d(x, y, Rotation2d.fromDegrees(gyroHeadingRadians)));
    odometryInitialized = true;
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
    leftThrottle = left;
    rightThrottle = right;
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

  public double getLeftDistanceMeters() {
    return leftFront.getEncoder().getPosition()
        * Constants.DriveMetrics.ENCODER_PPR
        * Constants.DriveMetrics.DISTANCE_PER_PULSE;
  }

  public double getOdoRight() {
    return rightFront.getEncoder().getPosition();
  }

  public double getRightDistanceMeters() {
    return rightFront.getEncoder().getPosition()
        * Constants.DriveMetrics.ENCODER_PPR
        * Constants.DriveMetrics.DISTANCE_PER_PULSE;
  }

  public void resetOdometry() {
    leftFront.getEncoder().setPosition(0.0);
    rightFront.getEncoder().setPosition(0.0);
  }

  public void passGyroHeadingRadians(float heading) {
    gyroHeadingRadians = heading;
  }

  /**
   * This can be used to pass the gyro heading to any autonomous command
   *
   * @return gyro heading in radians
   */
  public float getGyroHeadingRadians() {
    return gyroHeadingRadians;
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
    SmartDashboard.putNumber("TankDriveLeftPower", leftPower);
    SmartDashboard.putNumber("TankDriveRightPower", rightPower);
    differentialDrive.tankDrive(leftPower, rightPower);
  }

  public void arcadeDrive(double throttle, double turnRate) {
    differentialDrive.arcadeDrive(throttle, turnRate);
  }

  @Override
  public void periodic() {
    super.periodic();
    if (!autonomousRunning) {
      tankDrive(leftThrottle, rightThrottle);
    }
    if (odometryInitialized) {
      if (RobotBase.isReal()) {
        odometry.update(
            new Rotation2d(gyroHeadingRadians), getLeftDistanceMeters(), getRightDistanceMeters());
      } else {
        odometry.update(
            fakeGyro.getRotation2d(),
            fakeLeftEncoder.getDistance(),
            fakeRightEncoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(
        leftDriveGroup.get() * RobotController.getInputVoltage(),
        rightDriveGroup.get() * RobotController.getInputVoltage());
    drivetrainSim.update(0.02); // Robot loops every 20 milliseconds.
    leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-drivetrainSim.getHeading().getRadians());
  }

  public void initDefaultCommand() {
    setDefaultCommand(new ManualDriveCommand(this, this.driveController));
  }
}
