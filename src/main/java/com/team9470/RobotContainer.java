package com.team9470;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team9470.commands.AutoScoring;
import com.team9470.commands.Autos;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.Swerve;
import com.team9470.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Drivetrain and related commands remain unchanged
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final Swerve drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // ---------------- MECHANISM2D --------------------
    private final Mechanism2d mech = new Mechanism2d(5, 10);

    // Create the Superstructure instead of separate subsystems.
    private final Superstructure superstructure = new Superstructure(mech);
    private final AutoScoring autoScoring = new AutoScoring(drivetrain);

    // ----------------      VISION     --------------------
    private final Vision vision = Vision.getInstance();

    // ---------------- AUTONOMOUS --------------------
    private final Autos autos = new Autos(superstructure, drivetrain);
    private final AutoChooser autoChooser = new AutoChooser();

    CommandXboxController xbox = new CommandXboxController(0);
    Joystick buttonBoard = new Joystick(1);

    public RobotContainer() {
        configureBindings();

//        autoChooser.addRoutine("3CT", autos::getThreeCoralTop);
        autoChooser.addRoutine("3CTA", autos::getThreeCoralTopAutoAlign);
        autoChooser.addRoutine("3CTP", autos::getThreeCoralTopAutoPathing);
        autoChooser.addRoutine("3CBA", autos::getThreeCoralBottomAutoAlign);
        autoChooser.addRoutine("1C", autos::getOneCoral);
        autoChooser.addRoutine("1CC", autos::getOneCoralChoreo);
//        autoChooser.addRoutine("3CB Test", autos::getBottomThreeCoralTest);
        autoChooser.select("2C Test");
        SmartDashboard.putData("AutoChooser", autoChooser);
        SmartDashboard.putData("Mechanism", mech);

        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    public void periodic(){
        drivetrain.periodic();

    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-xbox.getLeftY() * MaxSpeed)
                                .withVelocityY(-xbox.getLeftX() * MaxSpeed)
                                .withRotationalRate(-xbox.getRightX() * MaxAngularRate)
                )
        );

        // Reset field-centric heading
        xbox.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // SUPERSTRUCTURE COMMANDS
        xbox.b().whileTrue(superstructure.reverseCoral());

        xbox.leftBumper().whileTrue(autoScoring.autoAlgaeNoDrive(superstructure))
                .onFalse(superstructure.getElevator().L0());

        // xbox.rightBumper().whileTrue(superstructure.groundIntake()).onFalse(superstructure.algaeReturn());
        xbox.rightBumper().whileTrue(superstructure.groundIntake());
        xbox.a().whileTrue(superstructure.processorScore()).onFalse(superstructure.algaeReturn());
        xbox.povRight().whileTrue(superstructure.raiseAndStow(2));
        xbox.back().onTrue(superstructure.triggerAlgaeHoming());

        // Example of binding elevator level commands via the superstructure's elevator
        for (int i = 0; i < 4; i++) {
            final int id = i;
            Trigger trig = new Trigger(() -> (id < 2) ? buttonBoard.getX() == Math.pow(-1.0, id + 1)
                    : buttonBoard.getY() == Math.pow(-1.0, id));
            trig.whileTrue(new InstantCommand(() -> autoScoring.overrideLevel(id + 1)));
        }

        // Reef position bindings (remaining unchanged)
        // for (int i = 0; i < 12; i++) {
        //     JoystickButton button = new JoystickButton(buttonBoard, i+1);
        //     final int id = i;
        //     button.whileTrue(new InstantCommand(() -> autoScoring.setBranch(id)));
        // }

        xbox.rightTrigger()
                .whileTrue(autoScoring.autoScore(superstructure))
                       // .onlyIf(superstructure.getCoral()::hasCoral))
                .onFalse(/*autoScoring.driveBack().andThen(*/superstructure.getElevator().L0()/*)*/);

        xbox.y()
                        .whileTrue(autoScoring.autoScoreNoDrive(superstructure).onlyIf(superstructure.getCoral()::hasCoral));

        // xbox.rightStick().whileTrue(new InstantCommand(autoScoring::updateClosestReefPos));
    }
}