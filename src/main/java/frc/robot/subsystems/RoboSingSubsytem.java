// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class RoboSingSubsytem extends SubsystemBase {
  private Orchestra m_orchestra;
  private TalonFX motor1;
  private TalonFX motor2;
  private TalonFX motor3;
  private TalonFX motor4;
  private TalonFX motor5;
  private TalonFX motor6;
  private TalonFX motor7;
  private TalonFX motor8;
  /** Creates a new RoboSing. */
  public RoboSingSubsytem() {
    m_orchestra = new Orchestra();


    //robo motors initilazied
    motor1 = new TalonFX(0);
    motor2 = new TalonFX(1);
    motor3 = new TalonFX(2);
    motor4 = new TalonFX(3);
    motor5 = new TalonFX(4);
    motor6 = new TalonFX(5);
    motor7 = new TalonFX(6);
    motor8 = new TalonFX(7);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Command playRock() {
  return runOnce(this::playRobotRock);
}

  
  public void playRobotRock( ) {
    String filePath = "roborock_test.chrp";
    m_orchestra.loadMusic(filePath);

    m_orchestra.addInstrument(motor1);
    m_orchestra.addInstrument(motor2);
    m_orchestra.addInstrument(motor3);
    m_orchestra.addInstrument(motor4);
    m_orchestra.addInstrument(motor5);
    m_orchestra.addInstrument(motor6);
    m_orchestra.addInstrument(motor7);
    m_orchestra.addInstrument(motor8);

    var status = m_orchestra.loadMusic(filePath);

    

    //if failed
    if (!status.isOK()) {
      System.out.println("Failed to load music: " + status.getDescription());
   }
  System.out.println("Music loaded successfully, starting playback.");

   m_orchestra.play();
  }
}
