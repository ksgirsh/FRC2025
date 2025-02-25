// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static class Robot{
        public static final double k_width = 26; // Inches
        public static final double k_length = 28; // Inches
    }


public static class Elevator {
    public static final int kElevatorLeftMotorId = 1;
    public static final int kElevatorRightMotorId = 2;

    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kIZone = 5.0;
    public static final double kG = 0.5;

    public static final double kMaxVelocity = 65;
    public static final double kMaxAcceleration = 200;

    public static final int kMaxCurrent = 40;

    public static final double kStowHeight = 0.0;
    public static final double kL2Height = 9.0;
    public static final double kL3Height = 25.14;
    public static final double kL4Height = 70.0;
    public static final double kMaxHeight = 56.2;
    public static final double kGroundAlgaeHeight = 0.0;
    public static final double kScoreAlgaeHeight = 0.0;
    public static final double kLowAlgaeHeight = 24.8;
    public static final double kHighAlgaeHeight = 42.5;
  }
  public static class AlgaeGroundtake {
    public static final int kPivotMotorId = 6;
    public static final int kIntakeMotorId = 7;

    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kIZone = 5.0;
    public static final double kG = 0.5;

    //speed of pivot
    public static final double kMaxVelocity = 20;
    public static final double kMaxAcceleration = 20;

    //speed of intake
    public static final double kIntakeSpeed = 0.3;
    public static final double kReverseSpeed = -0.3;

    public static final int kMaxCurrent = 40;
    // zerod at lying flat on insde of frame, prob gonna have to change later
    public static final double kInRotations = 0;
    public static final double kOutRotations = -3.0;
  }

  

  public static class Coral {
    public static final int kLeftMotorId = 3;
    public static final int kRightMotorId = 4;

    public static final int kLaserId = 0;
    public static final int kColorId = 16;

    public static final double kMaxCurrent = 20;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIZone = 0;

    public static final double kIntakeSpeed = 0.3;
    public static final double kReverseSpeed = -0.3;
    public static final double kL1Speed = 0.4;
    public static final double kL24Speed = 0.4;
    public static final double kIndexSpeed = 0.1;
    public static final double kSpeedDifference = kL1Speed * 0.5;
  }
}