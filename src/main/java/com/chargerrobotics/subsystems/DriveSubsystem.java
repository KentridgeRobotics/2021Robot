package com.chargerrobotics.subsystems;

import com.chargerrobotics.Constants;
import com.chargerrobotics.commands.drive.ManualDriveCommand;
import com.chargerrobotics.utils.CANSparkMaxSim;
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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
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
  private DifferentialDrive fakeDifferentialDrive;
  private DifferentialDriveKinematics driveKinematics;
  private SpeedControllerGroup leftDriveGroup;
  private SpeedControllerGroup rightDriveGroup;

  private DifferentialDriveOdometry odometry;

  private double leftThrottle;
  private double rightThrottle;

  private boolean brake;
  private boolean boost;
  private boolean slow;

  private boolean autonomousRunning;

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

    if (RobotBase.isSimulation()) {
      differentialDrive.close();
      leftRear = new CANSparkMaxSim(Constants.leftRearDrive, MotorType.kBrushless);
      leftFront = new CANSparkMaxSim(Constants.leftFrontDrive, MotorType.kBrushless);
      rightRear = new CANSparkMaxSim(Constants.rightRearDrive, MotorType.kBrushless);
      rightFront = new CANSparkMaxSim(Constants.rightFrontDrive, MotorType.kBrushless);
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
              LinearSystemId.identifyDrivetrainSystem(
                  Constants.DriveMetrics.KV_LINEAR,
                  Constants.DriveMetrics.KA_LINEAR,
                  Constants.DriveMetrics.KV_ANGULAR,
                  Constants.DriveMetrics.KA_ANGULAR),
              DCMotor.getNEO(2),
              Constants.DriveMetrics.DRIVE_GEAR_RATIO,
              Constants.DriveMetrics.TRACK_WIDTH_METERS,
              Constants.DriveMetrics.WHEEL_RADIUS_METERS,
              null); // Will not account for sensor measurement noise.
      field = new Field2d();
      SmartDashboard.putData("Field", field);
      leftDriveGroup = new SpeedControllerGroup(leftFront, leftRear);
      rightDriveGroup = new SpeedControllerGroup(rightFront, rightRear);
      fakeDifferentialDrive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
      fakeDifferentialDrive.setDeadband(0.0);
    }
    driveKinematics = new DifferentialDriveKinematics(Constants.DriveMetrics.TRACK_WIDTH_METERS);
    resetOdometry();
    odometry =
        new DifferentialDriveOdometry(
            new Rotation2d(
                getGyroHeadingRadians())); // the pose will always default to the origin with a
    // heading of 0 radians
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
    if (RobotBase.isReal()) {
      return leftFront.getEncoder().getPosition()
          * Constants.DriveMetrics.ENCODER_PPR
          * Constants.DriveMetrics.DISTANCE_PER_PULSE;
    }
    return fakeLeftEncoder.getDistance();
  }

  public double getOdoRight() {
    return rightFront.getEncoder().getPosition();
  }

  public double getRightDistanceMeters() {
    if (RobotBase.isReal()) {
      return rightFront.getEncoder().getPosition()
          * Constants.DriveMetrics.ENCODER_PPR
          * Constants.DriveMetrics.DISTANCE_PER_PULSE;
    }
    return fakeRightEncoder.getDistance();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftWheelSpeed = 0.0;
    double rightWheelSpeed = 0.0;
    if (RobotBase.isReal()) {
      leftWheelSpeed = // Converts motor RPM to the wheels' velocity in meters per second
          ((leftFront.getEncoder().getVelocity()
                      / Constants.DriveMetrics.MOTOR_ROTATIONS_PER_WHEEL_REV)
                  * Constants.DriveMetrics.WHEEL_CIRCUMFERENCE_METERS)
              / 60;
      rightWheelSpeed = // Converts motor RPM to the wheels' velocity in meters per second
          ((rightFront.getEncoder().getVelocity()
                      / Constants.DriveMetrics.MOTOR_ROTATIONS_PER_WHEEL_REV)
                  * Constants.DriveMetrics.WHEEL_CIRCUMFERENCE_METERS)
              / 60;
    } else {
      leftWheelSpeed = fakeLeftEncoder.getRate();
      rightWheelSpeed = fakeRightEncoder.getRate();
    }
    return new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
  }

  public void resetOdometry() {
    leftFront.getEncoder().setPosition(0.0);
    rightFront.getEncoder().setPosition(0.0);
  }

  public void resetPose(Pose2d pose) {
    resetOdometry();
    odometry.resetPosition(pose, new Rotation2d(getGyroHeadingRadians()));
    if (RobotBase.isSimulation()) {
      drivetrainSim.setPose(pose);
    }
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
    return RobotBase.isReal() ? gyroHeadingRadians : (float) -Math.toRadians(fakeGyro.getAngle());
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
    if (RobotBase.isReal()) {
      differentialDrive.arcadeDrive(throttle, turnRate);
    } else {
      fakeDifferentialDrive.arcadeDrive(turnRate, throttle); // yes this is intentional
    }
  }

  public void tankDriveVolts(double left, double right) {
    leftDriveGroup.setVoltage(left);
    rightDriveGroup.setVoltage(right);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getDriveKinematics() {
    return driveKinematics;
  }

  @Override
  public void periodic() {
    super.periodic();
    if (!autonomousRunning) {
      tankDrive(leftThrottle, rightThrottle);
    }
    odometry.update(
        new Rotation2d(getGyroHeadingRadians()), getLeftDistanceMeters(), getRightDistanceMeters());
    field.setRobotPose(odometry.getPoseMeters());
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
