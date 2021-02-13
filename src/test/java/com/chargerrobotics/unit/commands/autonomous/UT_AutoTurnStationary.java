package com.chargerrobotics.unit.commands.autonomous;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.reset;
import static org.mockito.Mockito.spy;
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
    
    
  /**
   * Test the 
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   * @param expectedValue The value that should be returned by as the reading
   */
  @Test(dataProvider = "Commands_AutoTurnStationary")
  public void testInitialization() {
    // Reset mock to ensure valid data
    reset(gyroMock);
    reset(driveMock);
    //when(internalControllerMock.getX(hand)).thenReturn(value);

    AutoTurnStationary testCommand = new AutoTurnStationary(driveMock, gyroMock);
    assertEquals(testCommand.isFinished(), false, "Command is initialized, but not finished."); //we want it to be false, if it is true then it has failed because it shouldn't have finished yet

    //assertNotEquals(joystickReading, 2.0, "Joystick reading not set!");
    //assertEquals(joystickReading, expectedValue, joystickAcceptableError);
  }
}
