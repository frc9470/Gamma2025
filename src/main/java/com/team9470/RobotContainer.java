// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team9470.commands.Autos;
import com.team9470.subsystems.AlgaeArm;
import com.team9470.subsystems.CoralManipulator;
import com.team9470.subsystems.Elevator;
import com.team9470.subsystems.Swerve;
import com.team9470.subsystems.vision.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;


public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public final Swerve drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // ---------------- MECHANISM2D --------------------
    private final Mechanism2d mech = new Mechanism2d(5, 10);

    // ---------------- SUBSYSTEMS --------------------
    private final Elevator elevator = new Elevator(mech);
    private final CoralManipulator coral = new CoralManipulator();
    private final AlgaeArm alg = new AlgaeArm(elevator.getElevatorLigament());

    // ----------------      VISION     --------------------
    private final Vision vision = Vision.getInstance();

    // ---------------- AUTONOMOUS --------------------

    private final Autos autos = new Autos(null, coral, elevator, drivetrain);
    private final AutoChooser autoChooser = new AutoChooser();

    CommandXboxController xbox = new CommandXboxController(0);
    Joystick buttonBoard = new Joystick(1);

    public RobotContainer() {

        configureBindings();

        autoChooser.addRoutine("4C Test", autos::getFourCoralTest);
        autoChooser.addRoutine("2C Test", autos::getTwoCoralTest);
        autoChooser.addRoutine("2C Optimized Test", autos::getTwoCoralOptimizedTest);
        autoChooser.select("4C Test");
        SmartDashboard.putData("AutoChooser", autoChooser);

        SmartDashboard.putData("Mechanism", mech);

        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-xbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

//        xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        xbox.b().whileTrue(new InstantCommand(() -> drivetrain.resetRotation(Rotation2d.fromDegrees(0))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xbox.back().and(xbox.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        xbox.back().and(xbox.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        xbox.start().and(xbox.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        xbox.start().and(xbox.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));


        xbox.leftBumper().whileTrue(elevator.L3().andThen(alg.deploy().alongWith(alg.spin()))).onFalse(alg.stow().andThen(elevator.L0()));
        xbox.rightBumper().whileTrue(elevator.L1().andThen(alg.deploy().alongWith(alg.reverse()))).onFalse(alg.stow().andThen(elevator.L0()));

        // reset the field-centric heading on left bumper press
        xbox.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // coral intake
        xbox.rightBumper().whileTrue(coral.scoreCommand());
        xbox.leftBumper().whileTrue(coral.reverseCommand());

        drivetrain.registerTelemetry(logger::telemeterize);

        xbox.rightTrigger().whileTrue(elevator.L4().andThen(coral.scoreCommand()).andThen(elevator.L0()));
//        xbox.x().whileTrue(elevator.L1().andThen(coral.scoreCommand()).andThen(elevator.L0()));
//        xbox.y().whileTrue(elevator.L2().andThen(coral.scoreCommand()).andThen(elevator.L0()));
//        xbox.b().whileTrue(elevator.L3().andThen(coral.scoreCommand()).andThen(elevator.L0()));

        xbox.leftTrigger().whileTrue(elevator.L3()
                .andThen(coral.scoreCommand())).onFalse(elevator.L0());

        //driverassist
//        drivetrain.getInstance();

        for(int i = 0; i < 12; i++){
                JoystickButton button = new JoystickButton(buttonBoard, i+1);
                button.whileTrue(drivetrain.pathfindReefPos(i));
        }
        JoystickButton button13 = new JoystickButton(buttonBoard, 13);
        button13.whileTrue(drivetrain.pathfindClosestReefPos());

    }

    public Command getAutonomousCommand(){
        return autoChooser.selectedCommand();
    }

}
