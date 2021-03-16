package com.chargerrobotics.subsystems;

import com.chargerrobotics.Constants;
import com.chargerrobotics.utils.NetworkMapping;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHoodSubsystem extends SubsystemBase {
    private static ShooterHoodSubsystem instance;
    private WPI_TalonSRX shooterHood;

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
        shooterHood.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.hoodPIDLoopId, Constants.hoodTimeOutMs);
        shooterHood.setNeutralMode(NeutralMode.Brake);
        shooterHood.configPeakCurrentLimit(40);
        shooterHood.configContinuousCurrentLimit(30);
        shooterHood.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    public void resetShooterEncoder() {
        shooterHood.setSelectedSensorPosition(0, Constants.hoodPIDLoopId, Constants.hoodTimeOutMs);
    }
    
    public void setHoodSpeed(double speed) {
        shooterHood.set(speed);
    }

    public double findHoodTargetTicks(double targetHoodAngDegrees) {
        double outPut = -34.217*targetHoodAngDegrees + 2532.087;
        return outPut;
    }

    public boolean isLimitSwitchTriggered() {
    	return shooterHood.getSensorCollection().isFwdLimitSwitchClosed();
        //return !shooterLimitSwitch.get();
    }

    public double getHoodPosition() {
        return (double)shooterHood.getSensorCollection().getQuadraturePosition();
    }
    
    public double getHoodAngle()  {
        LimelightSubsystem limelight = LimelightSubsystem.getInstance();
        double d = limelight.distance();
        //double d needs to be the information coming from the limelight, the length reported from the limelight
        double h = (Constants.targetHeight - Constants.cameraHeight);
        //conversion from inches to meters    
        double theta = Math.atan(2*h/d);
        //calculations for theta of equatio
    
        return Math.toDegrees(theta); 
    }

    public double getAngleByZone() {
        LimelightSubsystem limelight = LimelightSubsystem.getInstance();
        //double d = limelight.distance();
        double d = 50.0; //sample distance since limelight wasn't working
        // green zone -- 0 to 90 inches
        if (d >= 0 && d < 90) {
            //tested angle, seemed to work fine
            return 45.0;
        }
        // yellow zone
        if (d >= 90 && d < 150) {
            return 0.0;
        }
        // blue zone
        if (d >= 150 && d < 210) {
            return 0.0;
        }
        // red zone
        if (d >= 210 && d < 270) {
            return 0.0;
        }

        return 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (isLimitSwitchTriggered())
            resetShooterEncoder();
        SmartDashboard.putNumber("hoodCurrPos", shooterHood.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber("hood Current", shooterHood.getSupplyCurrent());
        SmartDashboard.putBoolean("HoodTriggered?", isLimitSwitchTriggered());
    }

}