package com.chargerrobotics.unit.commands.autonomous;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.reset;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import static org.testng.Assert.assertEquals;
import static org.testng.Assert.assertNotEquals;

import com.chargerrobotics.commands.autonomous.AutoTurnStationary;
import com.chargerrobotics.sensors.GyroscopeSerial;
import com.chargerrobotics.subsystems.DriveSubsystem;
import org.testng.annotations.BeforeClass;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;

public class UT_AutoTurnStationary {

    // Class to test with the unit test library
    private AutoTurnStationary commandAutoTurnStationary;

    // Classes that need to be mocked in order to support the command object
    // functionality without having a real drive or gyro component
    private  DriveSubsystem driveMock;
    private  GyroscopeSerial gyroMock; 


    @BeforeClass
    public void setUp() {
      
      // Init unit under test
      driveMock = mock(com.chargerrobotics.subsystems.DriveSubsystem.class);
      gyroMock = mock(com.chargerrobotics.sensors.GyroscopeSerial.class);

    }
    

  @Test()
  public void testStartTurning90Degrees() {
    // Reset mock to ensure valid data
    reset(gyroMock);
    reset(driveMock);
    //when (smartDashboardMock.getNumber("StationaryRotation", 90.0)).thenReturn(90.0);

    AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock, 90);
    testCommand.initialize();
    testCommand.execute();

    assertEquals(gyroMock.getHeading(), 0.0f);
    assertEquals(testCommand.GetRotationPID().getSetpoint(), 90.0, "GetRotationPID should return 90");
    boolean checkInitial = testCommand.GetLastRotationValue() >= 0.45;
    assertEquals(checkInitial, true, "Check Initial is false because value equals " + testCommand.GetLastRotationValue());
    

    // Add a gyro mock to change the heading
    reset(gyroMock);
    when (gyroMock.getHeading()).thenReturn(89.0f);

    // Run another iteration
    testCommand.execute();

    boolean checkNearTarget = testCommand.GetLastRotationValue() < 0.01 && testCommand.GetLastRotationValue() > -0.02; // sees if will move a little more, since we are already at 89
    assertEquals(checkNearTarget, true, "Check Near Target is false because value equals " + testCommand.GetLastRotationValue());


    reset(gyroMock);
    when (gyroMock.getHeading()).thenReturn(91.0f);

    // Run another iteration
    testCommand.execute();

    boolean checkAfterTarget = testCommand.GetLastRotationValue() < 0; 
    assertEquals(checkAfterTarget, true, "Check After Target is false because value equals " + testCommand.GetLastRotationValue());

  

    }
  
  
  @Test()
  public void Turn0Degrees() {
  
  reset(gyroMock);
  reset(driveMock);
  
  AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock, 0);
    testCommand.initialize();
    when (gyroMock.getHeading()).thenReturn(0.0f);
    testCommand.execute();

    assertEquals(gyroMock.getHeading(), 0.0f);
    assertEquals(testCommand.GetRotationPID().getSetpoint(), 0.0, "GetRotationPID should return 0");
    assertEquals(0.0,testCommand.GetLastRotationValue(), "Check Initial is false because value equals " + testCommand.GetLastRotationValue());
  }
  
  @Test()
  public void TurnNegative90Degrees() {
  
  reset(gyroMock);
  reset(driveMock);
  
  AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock, -90);
    testCommand.initialize();
    testCommand.execute();

    assertEquals(testCommand.GetRotationPID().getSetpoint(), -90.0, "GetRotationPID should return -90");
    boolean checkInitial = testCommand.GetLastRotationValue() <= -0.45;
    assertEquals(checkInitial, true, "Check Initial is false because value equals " + testCommand.GetLastRotationValue());
  }

  @Test()
  public void Turn90Degrees() {
  
  reset(gyroMock);
  reset(driveMock);
  
  AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock, 90);
    testCommand.initialize();
    testCommand.execute();

    assertEquals(testCommand.GetRotationPID().getSetpoint(), 90.0, "GetRotationPID should return 90");
    boolean checkInitial = testCommand.GetLastRotationValue() >= 0.45;
    assertEquals(checkInitial, true, "Check Initial is false because value equals " + testCommand.GetLastRotationValue());
  }

  @Test()
  public void TurnFrom89Degrees() {
  
  reset(gyroMock);
  reset(driveMock);
  
  AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock, 90);
    testCommand.initialize();    
    when (gyroMock.getHeading()).thenReturn(89.0f);
    testCommand.execute();

    boolean checkNearTarget = testCommand.GetLastRotationValue() < 0.01 && testCommand.GetLastRotationValue() > -0.02; // sees if will move a little more, since we are already at 89
    assertEquals(checkNearTarget, true, "Check Near Target is false because value equals " + testCommand.GetLastRotationValue());

  }
  
  @Test()
  public void TurnFrom91Degrees() {
  
  reset(gyroMock);
  reset(driveMock);
  
  AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock, 90);
    testCommand.initialize();    
    when (gyroMock.getHeading()).thenReturn(91.0f);
    testCommand.execute();

    boolean checkNearTarget = testCommand.GetLastRotationValue() < 0.01 && testCommand.GetLastRotationValue() > -0.02; // sees if will move a little more, since we are already at 89
    assertEquals(checkNearTarget, true, "Check Near Target is false because value equals " + testCommand.GetLastRotationValue());

  }


}
