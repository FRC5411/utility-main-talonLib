// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Libs.Utils;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.GenericHID;

public class XboxController extends GenericHID {
  /** Represents a digital button on an XboxController. */
  public enum Button {
    kLeftBumper(5),
    kRightBumper(6),
    kLeftStick(9),
    kRightStick(10),
    kA(1),
    kB(2),
    kX(3),
    kY(4),
    kBack(7),
    kStart(8);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and if not a Bumper button append `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Bumper")) {
        return name;
      }
      return name + "Button";
    }
  }

  /** Represents an axis on an XboxController. */
  public enum Axis {
    kLeftX(0),
    kRightX(4),
    kLeftY(1),
    kRightY(5),
    kLeftTrigger(2),
    kRightTrigger(3);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This is done by
     * stripping the leading `k`, and if a trigger axis append `Axis`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  public XboxController(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }

  public double getLeftX() {
    return getRawAxis(Axis.kLeftX.value);
  }

  public double getRightX() {
    return getRawAxis(Axis.kRightX.value);
  }

  public double getLeftY() {
    return getRawAxis(Axis.kLeftY.value);
  }

  public double getRightY() {
    return getRawAxis(Axis.kRightY.value);
  }

  public double getLeftTriggerAxis() {
    return getRawAxis(Axis.kLeftTrigger.value);
  }

  public double getRightTriggerAxis() {
    return getRawAxis(Axis.kRightTrigger.value);
  }

  public boolean getLeftBumper() {
    return getRawButton(Button.kLeftBumper.value);
  }

  public boolean getRightBumper() {
    return getRawButton(Button.kRightBumper.value);
  }

  public boolean getLeftBumperPressed() {
    return getRawButtonPressed(Button.kLeftBumper.value);
  }

  public boolean getRightBumperPressed() {
    return getRawButtonPressed(Button.kRightBumper.value);
  }

  public boolean getLeftBumperReleased() {
    return getRawButtonReleased(Button.kLeftBumper.value);
  }

  public boolean getRightBumperReleased() {
    return getRawButtonReleased(Button.kRightBumper.value);
  }

  public BooleanEvent leftBumper(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftBumper);
  }

  public BooleanEvent rightBumper(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightBumper);
  }

  public boolean getLeftStickButton() {
    return getRawButton(Button.kLeftStick.value);
  }

  public boolean getRightStickButton() {
    return getRawButton(Button.kRightStick.value);
  }

  public boolean getLeftStickButtonPressed() {
    return getRawButtonPressed(Button.kLeftStick.value);
  }

  public boolean getRightStickButtonPressed() {
    return getRawButtonPressed(Button.kRightStick.value);
  }

  public boolean getLeftStickButtonReleased() {
    return getRawButtonReleased(Button.kLeftStick.value);
  }

  public boolean getRightStickButtonReleased() {
    return getRawButtonReleased(Button.kRightStick.value);
  }

  public BooleanEvent leftStick(EventLoop loop) {
    return new BooleanEvent(loop, this::getLeftStickButton);
  }

  public BooleanEvent rightStick(EventLoop loop) {
    return new BooleanEvent(loop, this::getRightStickButton);
  }

  public boolean getAButton() {
    return getRawButton(Button.kA.value);
  }

  public boolean getAButtonPressed() {
    return getRawButtonPressed(Button.kA.value);
  }

  public boolean getAButtonReleased() {
    return getRawButtonReleased(Button.kA.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent a(EventLoop loop) {
    return new BooleanEvent(loop, this::getAButton);
  }

  public boolean getBButton() {
    return getRawButton(Button.kB.value);
  }

  public boolean getBButtonPressed() {
    return getRawButtonPressed(Button.kB.value);
  }

  public boolean getBButtonReleased() {
    return getRawButtonReleased(Button.kB.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent b(EventLoop loop) {
    return new BooleanEvent(loop, this::getBButton);
  }

  public boolean getXButton() {
    return getRawButton(Button.kX.value);
  }

  public boolean getXButtonPressed() {
    return getRawButtonPressed(Button.kX.value);
  }

  public boolean getXButtonReleased() {
    return getRawButtonReleased(Button.kX.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent x(EventLoop loop) {
    return new BooleanEvent(loop, this::getXButton);
  }

  public boolean getYButton() {
    return getRawButton(Button.kY.value);
  }

  public boolean getYButtonPressed() {
    return getRawButtonPressed(Button.kY.value);
  }

  public boolean getYButtonReleased() {
    return getRawButtonReleased(Button.kY.value);
  }

  @SuppressWarnings("MethodName")
  public BooleanEvent y(EventLoop loop) {
    return new BooleanEvent(loop, this::getYButton);
  }

  public boolean getBackButton() {
    return getRawButton(Button.kBack.value);
  }

  public boolean getBackButtonPressed() {
    return getRawButtonPressed(Button.kBack.value);
  }

  public boolean getBackButtonReleased() {
    return getRawButtonReleased(Button.kBack.value);
  }

  public BooleanEvent back(EventLoop loop) {
    return new BooleanEvent(loop, this::getBackButton);
  }

  public boolean getStartButton() {
    return getRawButton(Button.kStart.value);
  }

  public boolean getStartButtonPressed() {
    return getRawButtonPressed(Button.kStart.value);
  }

  public boolean getStartButtonReleased() {
    return getRawButtonReleased(Button.kStart.value);
  }

  public BooleanEvent start(EventLoop loop) {
    return new BooleanEvent(loop, this::getStartButton);
  }

  public BooleanEvent leftTrigger(double threshold, EventLoop loop) {
    return new BooleanEvent(loop, () -> getLeftTriggerAxis() > threshold);
  }

  public BooleanEvent leftTrigger(EventLoop loop) {
    return leftTrigger(0.5, loop);
  }

  public BooleanEvent rightTrigger(double threshold, EventLoop loop) {
    return new BooleanEvent(loop, () -> getRightTriggerAxis() > threshold);
  }

  public BooleanEvent rightTrigger(EventLoop loop) {
    return rightTrigger(0.5, loop);
  }
}
