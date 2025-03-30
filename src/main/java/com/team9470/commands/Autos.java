package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.*;
import com.team9470.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;

public class Autos extends SubsystemBase{
    private final AutoFactory autoFactory;
    private final AlgaeArm algaeArm;
    private final CoralManipulator coralManipulator;
    private final Elevator elevator;
    private final Swerve swerve;
    private final Superstructure superstructure;

    public Autos(Superstructure superstructure, Swerve swerve) {
        this.autoFactory = swerve.createAutoFactory((sample, isStart) -> {
        });
        this.algaeArm = superstructure.getAlgae();
        this.coralManipulator = superstructure.getCoral();
        this.elevator = superstructure.getElevator();
        this.swerve = swerve;
        this.superstructure = superstructure;
    }

    public static final double SCORING_DELAY = 0.3;
    public static final double INTAKE_DELAY = 0.;
    private static final double ELEVATOR_DELAY = 0.7;

    public Command scoreL4(){
        return elevator.L4().andThen(
                coralManipulator.scoreCommand().withTimeout(SCORING_DELAY))
                .andThen(elevator.L0());
    }

    public Command scoreL4WaitLower(Command driveAway, double delay){
        return elevator.L4()
                .andThen(
                    coralManipulator.scoreCommand().withTimeout(SCORING_DELAY)
                )
                .andThen(
                        driveAway
                        .alongWith(
                                new WaitCommand(delay).andThen(elevator.L0())
                        )
                );

    }

    public Command scoreL4AutoWaitLower(Command driveAway, double delay, int branchID){
        return AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(branchID, 4), swerve, SCORING_DELAY).andThen(driveAway
                .alongWith(
                        new WaitCommand(delay).andThen(elevator.L0())
                ));
    }

    public Command alignToSourceAndWait(boolean top) {
        Pose2d sourcePose = AllianceFlipUtil.apply(
                top ? new Pose2d(1.5841364860534668, 7.427209377288818, Rotation2d.fromDegrees(-54)) : new Pose2d(1.5463898181915283, 0.5990369319915771, Rotation2d.fromDegrees(54))
        );

        return new DriveToPose(() -> sourcePose, swerve, true, 5).raceWith(superstructure.waitForIntake());
    }

    public Command scoreCoral() {
        return coralManipulator.scoreCommand();
    }

    public AutoRoutine getOneCoralMiddleAutoNormal(){
        AutoRoutine routine = autoFactory.newRoutine("1CMN");

        AutoTrajectory startToC3 = routine.trajectory("S-3");
        routine.active().onTrue(
                startToC3.resetOdometry().andThen(
                        scoreL4AutoWaitLower(new InstantCommand(), SCORING_DELAY, 7)
                                .andThen(superstructure.algaeDown())
                )

        );
        return routine;
    }

    public AutoRoutine getOneCoralMiddleAutoChoreo(){
        AutoRoutine routine = autoFactory.newRoutine("1CMC");
        AutoTrajectory startToC3 = routine.trajectory("S-3");

        routine.active().onTrue(
                startToC3.resetOdometry()
                        .andThen(startToC3.cmd())
        );

        startToC3.done().onTrue(
                scoreL4WaitLower(new InstantCommand(), SCORING_DELAY)
                        .andThen(superstructure.algaeDown())
        );
        return routine;
    }

    public AutoRoutine getOneCoralMiddleAutoAlign(){
        AutoRoutine routine = autoFactory.newRoutine("1CMA");

        routine.active().onTrue(
                AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(7, 4), swerve)
                        .andThen(Commands.parallel(superstructure.algaeDown(), elevator.L0()))
        );
        
        return routine;
    }

    public AutoRoutine getFiveCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CTA");
        // TODO: define odom start position and reset odometry to that position, analogous to "startToC5.resetOdometry()" choreo call

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(9, 4), swerve, SCORING_DELAY),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSourceAndWait(true)),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, 4), swerve, SCORING_DELAY)),
                        elevator.L0().alongWith(algaeArm.down())
                                .withDeadline(alignToSourceAndWait(true)),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, 4), swerve, SCORING_DELAY)),
                        elevator.L0(),
                        alignToSourceAndWait(true),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, 3), swerve, SCORING_DELAY)),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSourceAndWait(true)),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, 3), swerve, SCORING_DELAY)),
                        elevator.L0(),
                        alignToSourceAndWait(true)
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CBA");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreStraight(superstructure, new AutoScoring.CoralObjective(4, 4), swerve, SCORING_DELAY),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSourceAndWait(false)),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 4), swerve, SCORING_DELAY)),
                        elevator.L0().alongWith(algaeArm.down())
                                .withDeadline(alignToSourceAndWait(false)),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 4), swerve, SCORING_DELAY)),
                        elevator.L0()
                                .withDeadline(alignToSourceAndWait(false)),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 3), swerve, SCORING_DELAY)),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSourceAndWait(false)),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 3), swerve, SCORING_DELAY)),
                        elevator.L0()
                                .withDeadline(alignToSourceAndWait(false))
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoAlignNoWait() {
        AutoRoutine routine = autoFactory.newRoutine("5CBA-NW");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreStraight(superstructure, new AutoScoring.CoralObjective(4, 4), swerve, SCORING_DELAY),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSourceAndWait(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 4), swerve, SCORING_DELAY),
                        elevator.L0().alongWith(algaeArm.down())
                                .withDeadline(alignToSourceAndWait(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 4), swerve, SCORING_DELAY),
                        elevator.L0()
                                .withDeadline(alignToSourceAndWait(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, 3), swerve, SCORING_DELAY),
                        elevator.L0().alongWith(algaeArm.up())
                                .withDeadline(alignToSourceAndWait(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, 3), swerve, SCORING_DELAY),
                        elevator.L0()
                                .withDeadline(alignToSourceAndWait(false)),
                        new WaitCommand(INTAKE_DELAY)
                )
        );

        return routine;
    }

    // basic leave auto
    public Command getBasicAutoCommand() {
        ChassisSpeeds initial_speed = new ChassisSpeeds(1, 0, 0);
        ChassisSpeeds final_speed = new ChassisSpeeds(0, 0, 0);

        return this.runEnd(
            () -> swerve.setChassisSpeeds(initial_speed),
            () -> swerve.setChassisSpeeds(final_speed)
        );
    }

}
