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
import com.chargerrobotics.utils.SmartDashboardHelper;
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
    private  SmartDashboardHelper smartDashboardMock;


    @BeforeClass
    public void setUp() {
      
      // Init unit under test
      driveMock = mock(com.chargerrobotics.subsystems.DriveSubsystem.class);
      gyroMock = mock(com.chargerrobotics.sensors.GyroscopeSerial.class);
      smartDashboardMock = mock (com.chargerrobotics.utils.SmartDashboardHelper.class);

    }
    
    
  /**
   * Test the 
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   * @param expectedValue The value that should be returned by as the reading
   */
  @Test()
  public void testStartTurning90Degrees() {
    // Reset mock to ensure valid data
    reset(gyroMock);
    reset(driveMock);
    reset(smartDashboardMock);
    //when (smartDashboardMock.getNumber("StationaryRotation", 90.0)).thenReturn(90.0);

    AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock, new SmartDashboardHelper());
    testCommand.SetTargetAngle(90);
    testCommand.initialize();
    testCommand.execute();

    assertEquals(gyroMock.getHeading(), 0.0f);
    assertEquals(testCommand.GetRotationPID().getSetpoint(), 90.0, "GetRotationPID should return 90");
    assertEquals(testCommand.GetLastRotationValue(), 229.59); // should be -1 to 1 ... ??

    // Add a gyro mock to change the heading
    reset(gyroMock);
    when (gyroMock.getHeading()).thenReturn(5.0f);

    // Run another iteration
    testCommand.execute();

    assertEquals(testCommand.GetLastRotationValue(), -8.075);

    // If you need to verify a mocked class ran function X times, this is how you do it
    // verify(gyroMock, times(1)).getHeading();

    
    // assert only occures when it fails, then it give this message to us to explain whats wrong
    //assertEquals(testCommand.isFinished(), false, "Command is initialized, but not finished."); //we want it to be false, if it is true then it has failed because it shouldn't have finished yet

    //assertNotEquals(joystickReading, 2.0, "Joystick reading not set!");
    //assertEquals(joystickReading, expectedValue, joystickAcceptableError);
  }
}
