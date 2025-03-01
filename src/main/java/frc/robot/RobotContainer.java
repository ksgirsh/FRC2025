// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


//subsystems imports
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.RoboSingSubsytem;
import frc.robot.subsystems.LimelightAlignment;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaeGroundtake;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //syom auto heading
    private double targetHeadingReef = 0; // in radians
    private double targetHeadingIntake = 0;


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    //syom auto heading
    private final SwerveRequest.FieldCentricFacingAngle headingRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05) // Add a 3% deadband to translation
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    ;

    

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);



    private final CommandXboxController driveJoystick = new CommandXboxController(0);

    private final CommandXboxController OperatorJoystick = new CommandXboxController(3);

    Joystick buttonBoard = new Joystick(1);
    private final JoystickButton back = new JoystickButton(buttonBoard, 10);
    private final JoystickButton right = new JoystickButton(buttonBoard, 11);
    private final JoystickButton forward = new JoystickButton(buttonBoard, 12);
    private final JoystickButton left = new JoystickButton(buttonBoard, 13);

    private final POVButton BbElevatorL4 = new POVButton(buttonBoard, 90);
    private final POVButton BbElevatorL3 = new POVButton(buttonBoard, 180);
    private final POVButton BbElevatorL2 = new POVButton(buttonBoard, 270);
    private final POVButton BbElevatorL1 = new POVButton(buttonBoard, 0);
    
    //initializing buttons, Bb stands for buttonboard
    private final JoystickButton BbReefBottomCenter = new JoystickButton(buttonBoard, 1);
    private final JoystickButton BbReefBottomRight = new JoystickButton(buttonBoard, 2);
    private final JoystickButton BbReefTopRight = new JoystickButton(buttonBoard, 3);
    private final JoystickButton BbReefTopCenter = new JoystickButton(buttonBoard, 4);
    private final JoystickButton BbReefTopLeft = new JoystickButton(buttonBoard, 5);
    private final JoystickButton BbReefBottomLeft = new JoystickButton(buttonBoard, 6);
    private final JoystickButton BbIntakeL = new JoystickButton(buttonBoard, 7);
    private final JoystickButton BbIntakeR = new JoystickButton(buttonBoard, 8);
    private final JoystickButton BbAlgaeProcessor = new JoystickButton(buttonBoard, 9);

    private final JoystickButton BbCoral1 = new JoystickButton(buttonBoard, 10);
    private final JoystickButton BbCoral2 = new JoystickButton(buttonBoard, 11);
    private final JoystickButton BbCoral3 = new JoystickButton(buttonBoard, 12);
     

    //instancing subsystems

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = Elevator.getInstance();
    public final AlgaeGroundtake algaeGroundtake = AlgaeGroundtake.getInstance();
    public final LimelightAlignment limelight = new LimelightAlignment();
    private final Coral coral = Coral.getInstance();



    //syom sing intialize
    public final RoboSingSubsytem roboSingSubsytem = new RoboSingSubsytem();
    // private final RoboSingCommand singCommand = new RoboSingCommand(roboSing);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        //syom sing example binding

        driveJoystick.pov(90).onTrue(roboSingSubsytem.playRock());


/*---------------------------------- driver joystick stuff----------------------------------*/


        // Set the default command for the drivetrain to be a lambda that sets the drive request
        // most basic drive commandd, left joystick is drives, right joystick is yaw
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // robot oriented drive forwad and backward, also left right
        driveJoystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driveJoystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        driveJoystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driveJoystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );

        //resets which which way is considered forward
        //the "back" button is in the middle of the controller slightly on the left
        driveJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //auto gyro heading PID syom
        headingRequest.HeadingController.setP(7.0);
        headingRequest.HeadingController.setI(0.0001);
        headingRequest.HeadingController.setD(1.2);

        //reef SET target heading
        BbReefBottomCenter.onTrue(
            drivetrain.runOnce(() -> {targetHeadingReef = 0; }) // close
        );
        BbReefBottomRight.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = Math.PI / 3) // close right
        );
        BbReefTopRight.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 2 * Math.PI / 3) // far right
        );
        BbReefTopCenter.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = Math.PI) // far
        );
        BbReefTopLeft.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 4 * Math.PI / 3) // far left
        );
        BbReefBottomLeft.onTrue(
            drivetrain.runOnce(() -> targetHeadingReef = 5 * Math.PI / 3) // close left
        );

        //intake station SET target heading
        BbIntakeL.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 2.2 +Math.PI) // left intake = face 126 degrees = 2.2 rads from forward 
        );
        BbIntakeR.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 4.084 + Math.PI) // right intake = face 234 degrees = 4.084 rads from forward
        );
        BbAlgaeProcessor.onTrue(
            drivetrain.runOnce(() -> targetHeadingIntake = 3 * Math.PI / 2) // algae processor = face right
        );

        //reef APPLY target heading
        driveJoystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(targetHeadingReef)))
        );
        //intake station APPLY target heading
        driveJoystick.leftTrigger().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(new Rotation2d(targetHeadingIntake)))
        );

        // Auto heading insta set and apply for four cardinal directions
        driveJoystick.y().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(0))) // Up
        );
        driveJoystick.b().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.PI / 2))) // Right
        );
        driveJoystick.a().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(Math.PI))) // Down
        );
        driveJoystick.x().whileTrue(
            drivetrain.applyRequest(() -> 
            headingRequest.withVelocityX(-driveJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driveJoystick.getLeftX() * MaxSpeed)
            .withTargetDirection(new Rotation2d(3 * Math.PI / 2))) // Left
        );


        // Limelight alignment
        driveJoystick.leftBumper().onTrue(limelight.setYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
        driveJoystick.leftBumper().whileTrue(limelight.LimelightAlign(drivetrain));

        //TODO make right bumper button allign to right reef branch 


/*---------------------------------- operator joystick and button board stuff----------------------------------*/
        

//elevator controls
        BbElevatorL4.onTrue(
            elevator.goToElevatorL4() // go to elevator level 4 of reef
        );
        BbElevatorL3.onTrue(
            elevator.goToElevatorL3() // go to elevator level 3 of reef
        );
        BbElevatorL2.onTrue(
            elevator.goToElevatorL2() // go to elevator level 2 of reef
        );
        BbElevatorL1.onTrue(
            elevator.goToElevatorStow() // go to elevator stow position
        );

//coral controls
        OperatorJoystick.y().whileTrue(
            coral.Intake() // runs the intake
        );
        OperatorJoystick.a().whileTrue(
            coral.LaserIntake() // runs the reverse intake
        );
        OperatorJoystick.b().whileTrue(
            coral.setSpeed(0.1, .05) // runs the L1
        );
        coral.setDefaultCommand(coral.stopIntake()); //whenever no button is pressed, intake doesnt spin


//algea groundtake controls
        OperatorJoystick.pov(0).onTrue(
            algaeGroundtake.goToPivotOut() // flops out the algea groundtake
        );
        OperatorJoystick.pov(180).onTrue(
            algaeGroundtake.goToPivotIn() // flops in the algea groundtake
        );
        OperatorJoystick.pov(90).whileTrue(
            algaeGroundtake.Intake() // runs the intake
        );
        OperatorJoystick.pov(270).whileTrue(
            algaeGroundtake.ReverseIntake() // runs the reverse intake
        );
        //whenever no button is pressed, intake doesnt spin
        algaeGroundtake.setDefaultCommand(algaeGroundtake.StopIntake());


//right bumper everythings resests to be compact
        OperatorJoystick.rightBumper().whileTrue(
            elevator.goToElevatorStow()
        );
        OperatorJoystick.rightBumper().whileTrue(
            algaeGroundtake.goToPivotIn()
        );
        



        





        // ong we dont need this shit wtf is breaking 

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));



        //wtf is a sysid bru

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}