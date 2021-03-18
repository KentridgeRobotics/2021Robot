package com.chargerrobotics.testutils;

import com.chargerrobotics.Constants;
import com.chargerrobotics.Robot;
import com.chargerrobotics.RobotContainer;
import com.chargerrobotics.utils.XboxController;
import com.chargerrobotics.utils.XboxController.WpiLibXboxControllerFactory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.testng.annotations.AfterMethod;
import org.testng.annotations.BeforeMethod;

public abstract class SimulatedTest {

  private Thread robotThread;

  protected RobotBase robot;
  private XboxControllerSim primarySim;
  private XboxControllerSim secondarySim;

  private edu.wpi.first.wpilibj.XboxController[] internalControllers =
      new edu.wpi.first.wpilibj.XboxController[2];

  class RobotSupplier implements Supplier<RobotBase> {

    private final RobotBase robot;

    public RobotSupplier(RobotBase robot) {
      this.robot = robot;
    }

    @Override
    public RobotBase get() {
      return robot;
    }
  }

  public class CommandConsumer implements Consumer<Command> {

    private Command targetCommand;

    private boolean completed;

    public CommandConsumer(Command targetCommand) {
      this.targetCommand = targetCommand;
      completed = false;
    }

    public boolean isCompleted() {
      return completed;
    }

    public void reset() {
      completed = false;
    }

    @Override
    public void accept(Command arg0) {
      if (arg0.getName().equals(targetCommand.getName())) {
        completed = true;
      }
    }
  }

  @BeforeMethod
  public void setUpSimulatedRobot() {
    robotThread =
        new Thread(
            () -> {
              robot.startCompetition();
            },
            "robot main");
    robotThread.setDaemon(true);

    robot = new Robot();

    // Update RobotContainer to have test controllers
    RobotContainer container = ((Robot) robot).getContainerInstance();

    internalControllers[0] = new edu.wpi.first.wpilibj.XboxController(Constants.primary);
    internalControllers[1] = new edu.wpi.first.wpilibj.XboxController(Constants.secondary);
    WpiLibXboxControllerFactory simControllerFactory =
        new WpiLibXboxControllerFactory() {

          @Override
          public edu.wpi.first.wpilibj.XboxController getController(int id) {
            return internalControllers[id];
          }
        };

    container.primary = XboxController.getInstance(Constants.primary, simControllerFactory);
    container.secondary = XboxController.getInstance(Constants.secondary, simControllerFactory);
    primarySim = new XboxControllerSim(internalControllers[0]);
    secondarySim = new XboxControllerSim(internalControllers[1]);
  }

  @AfterMethod
  public void cleanUpSimulatedRobot() {
    robot.endCompetition();
    if (robotThread.isAlive()) {
      robotThread.interrupt();
    }
    System.out.flush();

    DriverStationSim.setEnabled(false);
    XboxController.closeAll();
  }

  /** Begins running the robot in its test state, to be called just before testing anything. */
  protected void startRobot() {
    robotThread.start();
    RobotBase.suppressExitWarning(true);
    // Wait for robot to begin running
    while (!((Robot) robot).isInitialized()) {
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        break;
      }
    }
    DriverStationSim.setEnabled(true);
    try {
      // For some reason, this delay is needed to make the command scheduling
      // consistent.
      Thread.sleep(100);
    } catch (InterruptedException e) {
      return;
    }
  }

  /**
   * Get a {@link CommandConsumer} which is configured to mark when the given command is finished.
   *
   * @param command The {@link Command} to monitor for completion
   * @return The {@link CommandConsumer} which is notified when the {@link Command} is completed.
   */
  protected CommandConsumer getOnCommandFinished(Command command) {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();

    CommandConsumer commandConsumer = new CommandConsumer(command);
    scheduler.onCommandFinish(commandConsumer);

    return commandConsumer;
  }

  /**
   * Wait for the {@link CommandConsumer} to be notified that its command is finished or for the
   * timeout to expire
   *
   * @param commandConsumer The {@link CommandConsumer} to monitor
   * @param timeoutSeconds The number of seconds after which the wait should timeout
   * @return Whether or not the wait timed out
   */
  protected boolean waitForCommandFinish(CommandConsumer commandConsumer, int timeoutSeconds) {
    long start = System.currentTimeMillis();
    while (!commandConsumer.isCompleted()) {
      if (System.currentTimeMillis() > start + (timeoutSeconds * 1000)) {
        return true;
      }
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {
        break;
      }
    }

    return false;
  }
}
