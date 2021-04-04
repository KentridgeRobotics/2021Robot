package com.chargerrobotics.sensors;

import com.chargerrobotics.utils.ArduinoSerial;
import com.chargerrobotics.utils.ArduinoSerialReceiver;
import com.chargerrobotics.utils.ArduinoSerialReceiver.ArduinoListener;
import java.nio.ByteBuffer;

public class GyroscopeSerial extends ArduinoListener {

  private volatile float x = 0;
  private volatile float y = 0;
  private volatile float z = 0;

  public GyroscopeSerial() {
    ArduinoSerialReceiver.registerListener(this, (short) 0xA02D);
  }

  public void receiveData(ArduinoSerial serial, ByteBuffer buffer) {
    if (buffer.remaining() >= 12) {
      x = buffer.getFloat();
      y = buffer.getFloat();
      z = buffer.getFloat();
    }
  }

  /**
   * Gets the robot heading in degrees or {@code -1} if data is expired
   *
   * <p>Equivalent to {@link #getYaw()}
   *
   * @return Robot heading
   */
  public float getHeading() {
    return getYaw();
  }

  /**
   * Gets the robot heading in radians on the interval [-pi, pi) or {@code -9} if data is expired.
   * This method is counterclockwise-positive, but the gyro is clockwise positive
   *
   * @return Robot heading radians
   */
  public float getHeadingRadians() {
    float headingRad = x;
    if (headingRad > 180) {
      headingRad = 360 - x;
    } else {
      headingRad = -x;
    }
    return isExpired() ? -9 : (float) Math.toRadians(headingRad);
  }

  /**
   * Gets the robot yaw in degrees or {@code -1} if data is expired
   *
   * @return Robot yaw
   */
  public float getYaw() {
    return isExpired() ? -1 : x;
  }

  /**
   * Gets the robot heading without checking if the data is expired.
   *
   * @return Robot heading in degrees
   */
  public float getRawYaw() {
    return x;
  }

  /**
   * Gets the robot pitch in degrees or {@code -1} if data is expired
   *
   * @return Robot pitch
   */
  public float getPitch() {
    return isExpired() ? -1 : y;
  }

  /**
   * Gets the robot roll in degrees or {@code -1} if data is expired
   *
   * @return Robot roll
   */
  public float getRoll() {
    return isExpired() ? -1 : z;
  }
}
