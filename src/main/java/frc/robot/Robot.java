// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Libs.SProfile;
import frc.robot.Libs.Telemetry;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer timer;
  private SProfile profile;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Telemetry.setValue("Profile/Position", 0);
    Telemetry.setValue("Profile/Velocity", 0);
    Telemetry.setValue("Profile/Acceleration", 0);
    Telemetry.setValue("Profile/Jerk", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    timer = new Timer();
    profile = new SProfile(new SProfile.Constraints(250, 125, 25), 
    new SProfile.State(0, 0, 0, 0), new SProfile.State(1000, 0, 0, 0));
    timer.start();
  }

  @Override
  public void teleopPeriodic() {
    while(timer.get() < profile.totalTime()) {
      SProfile.State state = profile.calculate(timer.get());
      Telemetry.setValue("Profile/Position", state.position);
      Telemetry.setValue("Profile/Velocity", state.velocity);
      Telemetry.setValue("Profile/Acceleration", state.acceleration);
      Telemetry.setValue("Profile/Jerk", state.jerk);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
