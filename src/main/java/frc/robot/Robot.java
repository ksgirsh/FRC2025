// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private final RobotContainer m_robotContainer;
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  private Pigeon2 _pigeon;

  public Robot() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    // CameraServer.addCamera(new HttpCamera("limelight", "http://10.59.68.11:5800/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer));
    _pigeon = new Pigeon2(1, "rio");
  }

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    StatusSignal<Angle> yaw = _pigeon.getYaw();
    double robotYaw = -yaw.getValueAsDouble(); 
    SmartDashboard.putNumber("yaw", robotYaw);
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
  }

  @Override
  public void teleopPeriodic() {}

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

  @Override
  public void simulationPeriodic() {}
}
