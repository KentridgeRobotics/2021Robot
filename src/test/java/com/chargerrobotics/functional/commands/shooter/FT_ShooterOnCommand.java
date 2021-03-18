package com.chargerrobotics.functional.commands.shooter;

import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.testng.Assert.assertFalse;
import static org.testng.Assert.assertTrue;

import com.chargerrobotics.commands.shooter.ShooterOnCommand;
import com.chargerrobotics.subsystems.ShooterSubsystem;
import com.chargerrobotics.testutils.SimulatedTest;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.testng.annotations.BeforeMethod;
import org.testng.annotations.Test;

public class FT_ShooterOnCommand extends SimulatedTest {

  private ShooterSubsystem shooter;

  @BeforeMethod
  public void getShooterInstance() {
    shooter = ShooterSubsystem.getInstance();
  }

  @Test
  public void testShooterTurnsOn() {
    ShooterOnCommand command = spy(new ShooterOnCommand(shooter));
    CommandScheduler scheduler = CommandScheduler.getInstance();

    CommandConsumer consumer = getOnCommandFinished(command);
    startRobot();
    scheduler.schedule(command);

    boolean timedOut = waitForCommandFinish(consumer, 10);

    assertFalse(timedOut, "Timed out waiting for command to finish");
    verify(command, times(1)).initialize();
    assertTrue(shooter.isRunning(), "Shooter not running.");
  }
}
