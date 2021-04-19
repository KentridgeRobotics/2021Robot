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

import com.chargerrobotics.commands.autonomous.AutoDriveLinear;
import com.chargerrobotics.sensors.GyroscopeSerial;
import com.chargerrobotics.subsystems.DriveSubsystem;
import org.testng.annotations.BeforeClass;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;

public class UT_AutoDriveLinear {

     // Class to test with the unit test library
     private AutoDriveLinear commandAutoDriveLinear;

     // Classes that need to be mocked in order to support the command object
     private  DriveSubsystem driveMock;
     private  GyroscopeSerial gyroMock; 
 
 
     @BeforeClass
     public void setUp() {
       
       // Init unit under test
       driveMock = mock(com.chargerrobotics.subsystems.DriveSubsystem.class);
       gyroMock = mock(com.chargerrobotics.sensors.GyroscopeSerial.class);
 
     }
     

   @Test()
   public void testGo5outOf100() {
    reset(gyroMock);
    reset(driveMock);
    
    AutoDriveLinear testCommand = new AutoDriveLinear(driveMock, gyroMock, 100);
      testCommand.initialize();
      when (driveMock.getOdoLeft()).thenReturn(5.0);
      when (driveMock.getOdoRight()).thenReturn(5.0);
      testCommand.execute();
    

      assertEquals(testCommand.GetCurrentRightDistance(), 5.0, "CurrentRightDistance should be 5"); //making sure the values are set correctly  
      boolean checkWheelPower = testCommand.GetLastDriveValue() >= 0.4; //want it to be moving fowards, pretty fast, but exact speed varrys depending on values
      assertEquals(checkWheelPower, true, "GetLastDriveValue should return something higher than 0.4"); // the PID values should be at full power, or close to it because theres a lot to go

  }

   @Test()
   public void testGo100outOf100() {
    reset(gyroMock);
    reset(driveMock);
    
    AutoDriveLinear testCommand = new AutoDriveLinear(driveMock, gyroMock, 100);
      testCommand.initialize();
      when (driveMock.getOdoLeft()).thenReturn(100.0);
      when (driveMock.getOdoRight()).thenReturn(100.0);
      testCommand.execute();
    

      assertEquals(testCommand.GetCurrentRightDistance(), 100.0, "CurrentRightDistance should be 100"); 
      boolean checkWheelPower = testCommand.GetLastDriveValue() == 0.0;
      assertEquals(checkWheelPower, true, "GetLastDriveValue should return 0, as it should be stopped at this point"); 

  }
   
  @Test()
  public void testGo120outOf100() {
   reset(gyroMock);
   reset(driveMock);
   
   AutoDriveLinear testCommand = new AutoDriveLinear(driveMock, gyroMock, 100);
     testCommand.initialize();
     when (driveMock.getOdoLeft()).thenReturn(120.0);
     when (driveMock.getOdoRight()).thenReturn(120.0);
     testCommand.execute();
   

     assertEquals(testCommand.GetCurrentRightDistance(), 120.0, "CurrentRightDistance should be 120"); 
     boolean checkWheelPower = testCommand.GetLastDriveValue() < 0.0;
     assertEquals(checkWheelPower, true, "GetLastDriveValue should return something less than 0 because it overshot the goal, and should be going backwards"); 

 }

//Moving fowards, plus turning tests:

 @Test()
 public void testGo5outOf100_WheelEqual() {
  reset(gyroMock);
  reset(driveMock);
  
  AutoDriveLinear testCommand = new AutoDriveLinear(driveMock, gyroMock, 100);
    testCommand.initialize();
    when (driveMock.getOdoLeft()).thenReturn(5.0);
    when (driveMock.getOdoRight()).thenReturn(5.0);
    when (gyroMock.getHeading()).thenReturn(0.0f);

    testCommand.execute();
  
    //In previous tests we've been checking the LastDriveValue, but since we are looking to see the robot try to correct the uneven angle, now we will be looking at LastRotationValue
    assertEquals(testCommand.GetLastRotationValue(), 0.0, "The robot shouldn't be spining at all, since both wheels are equal."); 
    
}

 @Test()
 public void test10DegreesOffPath() {
  reset(gyroMock);
  reset(driveMock);
  
  AutoDriveLinear testCommand = new AutoDriveLinear(driveMock, gyroMock, 100);
    testCommand.initialize();
    when (gyroMock.getHeading()).thenReturn(10.0f); 
    testCommand.execute();
     
    boolean checkRotationValue = testCommand.GetLastRotationValue() < 0.0;
    assertEquals(checkRotationValue, true, "Value should be under 0, in order to move robot back to straight"); 

  }

 
  @Test()
  public void testNegative10DegreesOffPath() {
   reset(gyroMock);
   reset(driveMock);
   
   AutoDriveLinear testCommand = new AutoDriveLinear(driveMock, gyroMock, 100);
     testCommand.initialize();
     when (gyroMock.getHeading()).thenReturn(-10.0f); 
     testCommand.execute();
       
     boolean checkRotationValue = testCommand.GetLastRotationValue() > 0.0;
     assertEquals(checkRotationValue, true, "Value should be greater than 0, in order to move robot back to straight"); 
 
   }
    //Testing for both the speed for moving fowards and the speed for turning back to a straight angle 
    @Test()
    public void testGo50outOf100_Wheel5DegreesOff() {
     reset(gyroMock);
     reset(driveMock);
     
     AutoDriveLinear testCommand = new AutoDriveLinear(driveMock, gyroMock, 100);
       testCommand.initialize();
       when (driveMock.getOdoLeft()).thenReturn(50.0);
       when (driveMock.getOdoRight()).thenReturn(50.0);
       when (gyroMock.getHeading()).thenReturn(15.0f);
       testCommand.execute();
     
       //Checking moving and distance values
       assertEquals(testCommand.GetCurrentRightDistance(), 50.0, "CurrentRightDistance should be 50"); //just checking the right side to make sure the correct values were passed through and where not changed
       boolean checkWheelPower = testCommand.GetLastDriveValue() > 0.2;
       assertEquals(checkWheelPower, true, "GetLastDriveValue should return something higher than 0.2 because it still should be going forwards, at a quick speed"); 
      //Checking angle and gyro values
       boolean checkRotationValue = testCommand.GetLastRotationValue() < 0.0;
       assertEquals(checkRotationValue, true, "Value should be under 0, in order to move robot back to straight"); 
   
   }
}
