package com.chargerrobotics.utils;

import com.chargerrobotics.utils.XboxPovButton.POVDirection;
import com.google.common.annotations.VisibleForTesting;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.HashMap;

public class XboxController {

  public interface WpiLibXboxControllerFactory {
    public edu.wpi.first.wpilibj.XboxController getController(int id);
  }

  private static final HashMap<Integer, XboxController> instances =
      new HashMap<Integer, XboxController>();

  edu.wpi.first.wpilibj.XboxController controller;

  public JoystickButton buttonA;
  public JoystickButton buttonB;
  public JoystickButton buttonX;
  public JoystickButton buttonY;
  public JoystickButton buttonBumperLeft;
  public JoystickButton buttonBumperRight;
  public JoystickButton buttonView;
  public JoystickButton buttonMenu;
  public JoystickButton buttonStickLeft;
  public JoystickButton buttonStickRight;
  public XboxPovButton buttonPovUp;
  public XboxPovButton buttonPovRight;
  public XboxPovButton buttonPovDown;
  public XboxPovButton buttonPovLeft;

  private static final double deadzone = 0.13;

  private XboxController(edu.wpi.first.wpilibj.XboxController controller) {
    this.controller = controller;
    setupButtons();
  }

  /**
   * Convenience method for getting controller ID 0
   *
   * @return Controller with ID 0
   */
  public static XboxController getInstance() {
    return getInstance(0);
  }

  /**
   * Get instance of the controller with specified ID
   *
   * @param id The ID of the controller to get
   * @return Controller with the given ID
   */
  public static XboxController getInstance(int id) {
    return getInstance(
        id,
        new WpiLibXboxControllerFactory() {
          @Override
          public edu.wpi.first.wpilibj.XboxController getController(int id) {
            return new edu.wpi.first.wpilibj.XboxController(id);
          }
        });
  }

  /**
   * Get instance of the controller with specified ID and use provided factory to create a
   * controller. This is intended for testing purposes
   *
   * @param id The ID of the controller to get
   * @param WpiLibXboxControllerFactory A factory to create the internal controller to use
   * @return Controller with the given ID
   */
  @VisibleForTesting
  public static XboxController getInstance(int id, WpiLibXboxControllerFactory controllerFactory) {
    XboxController inst = instances.get(id);
    if (inst == null) {
      instances.put(id, new XboxController(controllerFactory.getController(id)));
    }
    return instances.get(id);
  }

  public static void closeAll() {
    instances.clear();
  }

  private void setupButtons() {
    buttonA = new JoystickButton(controller, XboxControllerButton.A.getId());
    buttonB = new JoystickButton(controller, XboxControllerButton.B.getId());
    buttonX = new JoystickButton(controller, XboxControllerButton.X.getId());
    buttonY = new JoystickButton(controller, XboxControllerButton.Y.getId());
    buttonBumperLeft = new JoystickButton(controller, XboxControllerButton.BUMPER_LEFT.getId());
    buttonBumperRight = new JoystickButton(controller, XboxControllerButton.BUMPER_RIGHT.getId());
    buttonView = new JoystickButton(controller, XboxControllerButton.VIEW.getId());
    buttonMenu = new JoystickButton(controller, XboxControllerButton.MENU.getId());
    buttonStickLeft = new JoystickButton(controller, XboxControllerButton.STICK_LEFT.getId());
    buttonStickRight = new JoystickButton(controller, XboxControllerButton.STICK_RIGHT.getId());
    buttonPovUp = new XboxPovButton(controller, POVDirection.UP);
    buttonPovRight = new XboxPovButton(controller, POVDirection.RIGHT);
    buttonPovDown = new XboxPovButton(controller, POVDirection.DOWN);
    buttonPovLeft = new XboxPovButton(controller, POVDirection.LEFT);
  }

  private double correctForDeadzone(double raw) {
    if (Math.abs(raw) <= deadzone) return 0.0;
    else {
      double c = (Math.abs(raw) - deadzone) / (1.0 - deadzone);
      return (raw > 0) ? c : -c;
    }
  }

  public double getLeftStickX() {
    double n = controller.getX(Hand.kLeft);
    return correctForDeadzone(n);
  }

  public double getLeftStickY() {
    double n = controller.getY(Hand.kLeft);
    return correctForDeadzone(n);
  }

  public double getRightStickX() {
    double n = controller.getX(Hand.kRight);
    return correctForDeadzone(n);
  }

  public double getRightStickY() {
    double n = controller.getY(Hand.kRight);
    return correctForDeadzone(n);
  }

  public double getLeftTrigger() {
    return controller.getTriggerAxis(Hand.kLeft);
  }

  public double getRightTrigger() {
    return controller.getTriggerAxis(Hand.kRight);
  }

  public enum XboxControllerButton {
    A(1),
    B(2),
    X(3),
    Y(4),
    BUMPER_LEFT(5),
    BUMPER_RIGHT(6),
    VIEW(7),
    MENU(8),
    STICK_LEFT(9),
    STICK_RIGHT(10);

    private final int id;

    XboxControllerButton(int id) {
      this.id = id;
    }

    public int getId() {
      return id;
    }
  }
}
