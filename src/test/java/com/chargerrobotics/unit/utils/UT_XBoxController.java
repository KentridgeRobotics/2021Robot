package com.chargerrobotics.unit.utils;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.reset;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.when;
import static org.testng.Assert.assertEquals;
import static org.testng.Assert.assertNotEquals;

import com.chargerrobotics.utils.XboxController;
import com.chargerrobotics.utils.XboxController.WpiLibXboxControllerFactory;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import org.testng.annotations.BeforeClass;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;

public class UT_XBoxController {

  private static final double joystickAcceptableError = 0.000001;

  private com.chargerrobotics.utils.XboxController controllerSpy;

  private edu.wpi.first.wpilibj.XboxController internalControllerMock;

  @BeforeClass
  public void setUp() {
    XboxController.closeAll();

    // Init unit under test
    internalControllerMock = mock(edu.wpi.first.wpilibj.XboxController.class);

    // Provide a controller factory so that the singleton uses the mocked controller
    controllerSpy =
        spy(
            XboxController.getInstance(
                0,
                new WpiLibXboxControllerFactory() {
                  @Override
                  public edu.wpi.first.wpilibj.XboxController getController(int id) {
                    return internalControllerMock;
                  }
                }));
  }

  /**
   * Tests that when a particular X value is returned from the controller that the expected reading
   * is provided
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   * @param expectedValue The value that should be returned by as the reading
   */
  @Test(dataProvider = "joystickResultsProvider")
  public void testStickX(double value, Hand hand, double expectedValue) {
    // Reset mock to ensure valid data
    reset(internalControllerMock);
    when(internalControllerMock.getX(hand)).thenReturn(value);

    double joystickReading =
        2.0; // Utilize 2.0 as an invalid value as it is out of the joystick range

    switch (hand) {
      case kLeft:
        joystickReading = controllerSpy.getLeftStickX();
        break;
      case kRight:
        joystickReading = controllerSpy.getRightStickX();
        break;
    }

    assertNotEquals(joystickReading, 2.0, "Joystick reading not set!");
    assertEquals(joystickReading, expectedValue, joystickAcceptableError);
  }

  /**
   * Tests that when a particular Y value is returned from the controller that the expected reading
   * is provided
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   * @param expectedValue The value that should be returned by as the reading
   */
  @Test(dataProvider = "joystickResultsProvider")
  public void testStickY(double value, Hand hand, double expectedValue) {
    when(internalControllerMock.getY(hand)).thenReturn(value);

    double joystickReading =
        2.0; // Utilize 2.0 as an invalid value as it is out of the joystick range
    switch (hand) {
      case kLeft:
        joystickReading = controllerSpy.getLeftStickY();
        break;
      case kRight:
        joystickReading = controllerSpy.getRightStickY();
        break;
    }

    assertNotEquals(joystickReading, 2.0, "Joystick reading not set!");
    assertEquals(joystickReading, expectedValue, joystickAcceptableError);
  }

  @DataProvider
  public Object[][] joystickResultsProvider() {
    return new Object[][] {
      {0.0, Hand.kLeft, 0.0},
      {0.0, Hand.kRight, 0.0},
      {0.05, Hand.kLeft, 0.0},
      {0.05, Hand.kRight, 0.0},
      {0.10, Hand.kLeft, 0.0},
      {0.10, Hand.kRight, 0.0},
      {0.13, Hand.kLeft, 0.0},
      {0.13, Hand.kRight, 0.0},
      {0.111111111111, Hand.kLeft, 0.0},
      {0.111111111111, Hand.kRight, 0.0},
      {-0.0, Hand.kLeft, 0.0},
      {-0.0, Hand.kRight, 0.0},
      {-0.05, Hand.kLeft, 0.0},
      {-0.05, Hand.kRight, 0.0},
      {-0.10, Hand.kLeft, 0.0},
      {-0.10, Hand.kRight, 0.0},
      {-0.13, Hand.kLeft, 0.0},
      {-0.13, Hand.kRight, 0.0},
      {-0.111111111111, Hand.kLeft, 0.0},
      {-0.111111111111, Hand.kRight, 0.0},
      {1.0, Hand.kLeft, 1.0},
      {1.0, Hand.kRight, 1.0},
      {0.5, Hand.kLeft, 0.425287},
      {0.5, Hand.kRight, 0.425287},
      {0.25, Hand.kLeft, 0.137931},
      {0.25, Hand.kRight, 0.137931},
      {0.2, Hand.kLeft, 0.080460},
      {0.2, Hand.kRight, 0.080460},
      {0.13000000001, Hand.kLeft, 0.00000000001},
      {0.13000000001, Hand.kRight, 0.00000000001},
      {-1.0, Hand.kLeft, -1.0},
      {-1.0, Hand.kRight, -1.0},
      {-0.5, Hand.kLeft, -0.425287},
      {-0.5, Hand.kRight, -0.425287},
      {-0.25, Hand.kLeft, -0.137931},
      {-0.25, Hand.kRight, -0.137931},
      {-0.2, Hand.kLeft, -0.080460},
      {-0.2, Hand.kRight, -0.080460},
      {-0.13000000001, Hand.kLeft, -0.00000000001},
      {-0.13000000001, Hand.kRight, -0.00000000001},
    };
  }
}
