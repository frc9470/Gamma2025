package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.*;
import com.team9470.commands.AutoScoring;
import com.team9470.util.AllianceFlipUtil;
import com.team9470.util.LogUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public Command alignToSourceAndWait() {
        Pose2d sourcePose = AllianceFlipUtil.apply(
                new Pose2d(1.629029631614685, 7.376053314208984, Rotation2d.fromDegrees(-54))
        );

        return new DriveToPose(() -> sourcePose, swerve)/*.withDeadline(superstructure.waitForIntake())*/;
    }

    public Command scoreCoral() {
        return coralManipulator.scoreCommand();
    }


    public AutoRoutine getThreeCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("3CTA");

        // Trajectories
        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1", 1);
        AutoTrajectory toC12 = routine.trajectory("TC-12", 0);
        AutoTrajectory C12toSource = routine.trajectory("TC-12", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);

        LogUtil.recordPose2d("autostart", startToC1.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        startToC1.resetOdometry(),
                        startToC1.cmd()
                )
        );

        startToC1.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        startToC1.done().onTrue(
                scoreL4AutoWaitLower(C1toSource.cmd(), SCORING_DELAY, 9)
        );

        C1toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC12.cmd())
        );

        toC12.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );


        toC12.done().onTrue(
                scoreL4AutoWaitLower(C12toSource.cmd(), SCORING_DELAY, 10)
        );

        C12toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC11.cmd())
        );

        toC11.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC11.done().onTrue(
                scoreL4AutoWaitLower(C11toSource.cmd(), SCORING_DELAY, 11)
        );

        return routine;
    }

    public AutoRoutine getThreeCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("3CBA");

        // Trajectories
        AutoTrajectory startToC5 = routine.trajectory("S-5");
        AutoTrajectory C5toSource = routine.trajectory("BC-5", 1);
        AutoTrajectory toC7 = routine.trajectory("BC-7", 0);
        AutoTrajectory C7toSource = routine.trajectory("BC-7", 1);
        AutoTrajectory toC8 = routine.trajectory("BC-8", 0);
        AutoTrajectory C8toSource = routine.trajectory("BC-8", 1);

        LogUtil.recordPose2d("autostart", startToC5.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        startToC5.resetOdometry(),
                        startToC5.cmd()
                )
        );

        startToC5.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        startToC5.done().onTrue(
                scoreL4AutoWaitLower(C5toSource.cmd(), SCORING_DELAY, 5)
        );

        C5toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC7.cmd())
        );

        toC7.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC7.done().onTrue(
                scoreL4AutoWaitLower(C7toSource.cmd(), SCORING_DELAY, 3)
        );

        C7toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC8.cmd())
        );

        toC8.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC8.done().onTrue(
                scoreL4AutoWaitLower(C8toSource.cmd(), SCORING_DELAY, 2)
        );

        return routine;
    }



    public AutoRoutine getThreeCoralTopAutoPathing() {
        AutoRoutine routine = autoFactory.newRoutine("3CTP");

        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1back = routine.trajectory("TC-1", 1);
        AutoTrajectory to11Score = routine.trajectory("SCORE 11 P");
        AutoTrajectory to12Score = routine.trajectory("SCORE 12 P");
        AutoTrajectory C11back = routine.trajectory("TC-11", 1);
        AutoTrajectory C12back = routine.trajectory("TC-12", 1);


        routine.active().onTrue(
                Commands.sequence(
                        startToC1.resetOdometry(),
                        startToC1.cmd()
                )
        );

        startToC1.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );
        startToC1.done().onTrue(
                scoreL4AutoWaitLower(C1back.cmd(), SCORING_DELAY, 9)
        );

        C1back.done().onTrue(
                alignToSourceAndWait().andThen(to11Score.cmd())
        );

        to11Score.done().onTrue(
                scoreL4AutoWaitLower(C11back.cmd(), SCORING_DELAY, 10)
        );

        C11back.done().onTrue(
                alignToSourceAndWait().andThen(to12Score.cmd())
        );

        to12Score.done().onTrue(
                scoreL4AutoWaitLower(C12back.cmd(), SCORING_DELAY, 11)
        );
        return routine;
    }

    public AutoRoutine getOneCoralMiddleAutoNormal(){
        AutoRoutine routine = autoFactory.newRoutine("1CMN");

        AutoTrajectory startToC3 = routine.trajectory("S-3");
        routine.active().onTrue(
                startToC3.resetOdometry().andThen(
                        scoreL4AutoWaitLower(new InstantCommand(), SCORING_DELAY, 7)
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
        );
        return routine;
    }

    public AutoRoutine getFourCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("4CTA");

        // Trajectories
        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1", 1);
        AutoTrajectory toC12 = routine.trajectory("TC-12", 0);
        AutoTrajectory C12toSource = routine.trajectory("TC-12", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);

        LogUtil.recordPose2d("autostart", startToC1.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        startToC1.resetOdometry(),
                        startToC1.cmd()
                )
        );

        startToC1.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        startToC1.done().onTrue(
                scoreL4AutoWaitLower(C1toSource.cmd(), SCORING_DELAY, 9)
        );

        C1toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC12.cmd())
        );

        toC12.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC12.done().onTrue(
                scoreL4AutoWaitLower(C12toSource.cmd(), SCORING_DELAY, 10)
        );

        C12toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC11.cmd())
        );

        toC11.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC11.done().onTrue(
                scoreL4AutoWaitLower(C11toSource.cmd(), SCORING_DELAY, 11)
        );

        C11toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC10.cmd())
        );

        toC10.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC10.done().onTrue(
                scoreL4AutoWaitLower(C10toSource.cmd(), SCORING_DELAY, 0)
        );

        return routine;
    }

    public AutoRoutine getFourCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("4CBA");

        // Trajectories
        AutoTrajectory startToC5 = routine.trajectory("S-5");
        AutoTrajectory C5toSource = routine.trajectory("BC-5", 1);
        AutoTrajectory toC7 = routine.trajectory("BC-7", 0);
        AutoTrajectory C7toSource = routine.trajectory("BC-7", 1);
        AutoTrajectory toC8 = routine.trajectory("BC-8", 0);
        AutoTrajectory C8toSource = routine.trajectory("BC-8", 1);
        AutoTrajectory toC9 = routine.trajectory("BC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("BC-9", 1);

        LogUtil.recordPose2d("autostart", startToC5.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        startToC5.resetOdometry(),
                        startToC5.cmd()
                )
        );

        startToC5.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        startToC5.done().onTrue(
                scoreL4AutoWaitLower(C5toSource.cmd(), SCORING_DELAY, 5)
        );

        C5toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC7.cmd())
        );

        toC7.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC7.done().onTrue(
                scoreL4AutoWaitLower(C7toSource.cmd(), SCORING_DELAY, 3)
        );

        C7toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC8.cmd())
        );

        toC8.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC8.done().onTrue(
                scoreL4AutoWaitLower(C8toSource.cmd(), SCORING_DELAY, 2)
        );

        C8toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC9.cmd())
        );

        toC9.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC9.done().onTrue(
                scoreL4AutoWaitLower(C9toSource.cmd(), SCORING_DELAY, 1)
        );

        return routine;
    }

    public AutoRoutine getFiveCoralTopAutoNormal() {
        AutoRoutine routine = autoFactory.newRoutine("5CTN");

        // Trajectories
        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1", 1);
        AutoTrajectory toC12 = routine.trajectory("TC-12", 0);
        AutoTrajectory C12toSource = routine.trajectory("TC-12", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);
        AutoTrajectory toC9 = routine.trajectory("TC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("TC-9", 1);

        LogUtil.recordPose2d("autostart", startToC1.getInitialPose().get());

        // Start segment with reset odometry and first command.
        routine.active().onTrue(
                Commands.sequence(
                        startToC1.resetOdometry(),
                        startToC1.cmd()
                )
        );
        startToC1.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(elevator.L4());
        startToC1.done().onTrue(
                scoreL4AutoWaitLower(C1toSource.cmd(), SCORING_DELAY, 9)
        );
        C1toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC12.cmd())
        );

        // Chain segments using helper method.
        attachSegment(toC12, C12toSource, 10, toC11);
        attachSegment(toC11, C11toSource, 11, toC10);
        attachSegment(toC10, C10toSource, 0, toC9);
        attachSegment(toC9, C9toSource, 1, null);

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoNormal() {
        AutoRoutine routine = autoFactory.newRoutine("5CBN");

        // Trajectories
        AutoTrajectory startToC5 = routine.trajectory("S-5");
        AutoTrajectory C5toSource = routine.trajectory("BC-5", 1);
        AutoTrajectory toC7 = routine.trajectory("BC-7", 0);
        AutoTrajectory C7toSource = routine.trajectory("BC-7", 1);
        AutoTrajectory toC8 = routine.trajectory("BC-8", 0);
        AutoTrajectory C8toSource = routine.trajectory("BC-8", 1);
        AutoTrajectory toC9 = routine.trajectory("BC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("BC-9", 1);
        AutoTrajectory toC10 = routine.trajectory("BC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("BC-10", 1);

        LogUtil.recordPose2d("autostart", startToC5.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        startToC5.resetOdometry(),
                        startToC5.cmd()
                )
        );

        startToC5.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        startToC5.done().onTrue(
                scoreL4AutoWaitLower(C5toSource.cmd(), SCORING_DELAY, 5)
        );

        C5toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC7.cmd())
        );

        toC7.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC7.done().onTrue(
                scoreL4AutoWaitLower(C7toSource.cmd(), SCORING_DELAY, 3)
        );

        C7toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC8.cmd())
        );

        toC8.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC8.done().onTrue(
                scoreL4AutoWaitLower(C8toSource.cmd(), SCORING_DELAY, 2)
        );

        C8toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC9.cmd())
        );

        toC9.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC9.done().onTrue(
                scoreL4AutoWaitLower(C9toSource.cmd(), SCORING_DELAY, 1)
        );

        C9toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC10.cmd())
        );

        toC10.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC10.done().onTrue(
                scoreL4AutoWaitLower(C10toSource.cmd(), SCORING_DELAY, 0)
        );

        return routine;
    }

    public AutoRoutine getFiveCoralTopAutoChoreo() {
        AutoRoutine routine = autoFactory.newRoutine("5CTC");

        // Trajectories
        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1", 1);
        AutoTrajectory toC12 = routine.trajectory("TC-12", 0);
        AutoTrajectory C12toSource = routine.trajectory("TC-12", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);
        AutoTrajectory toC9 = routine.trajectory("TC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("TC-9", 1);

        LogUtil.recordPose2d("autostart", startToC1.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        startToC1.resetOdometry(),
                        startToC1.cmd()
                )
        );

        startToC1.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        startToC1.done().onTrue(
                scoreL4WaitLower(C1toSource.cmd(), SCORING_DELAY)
        );

        C1toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC12.cmd())
        );

        toC12.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC12.done().onTrue(
                scoreL4WaitLower(C12toSource.cmd(), SCORING_DELAY)
        );

        C12toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC11.cmd())
        );

        toC11.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC11.done().onTrue(
                scoreL4WaitLower(C11toSource.cmd(), SCORING_DELAY)
        );

        C11toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC10.cmd())
        );

        toC10.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC10.done().onTrue(
                scoreL4WaitLower(C10toSource.cmd(), SCORING_DELAY)
        );

        C10toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC9.cmd())
        );

        toC9.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC9.done().onTrue(
                scoreL4WaitLower(C9toSource.cmd(), SCORING_DELAY)
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoChoreo() {
        AutoRoutine routine = autoFactory.newRoutine("5CBC");

        // Trajectories
        AutoTrajectory startToC5 = routine.trajectory("S-5");
        AutoTrajectory C5toSource = routine.trajectory("BC-5", 1);
        AutoTrajectory toC7 = routine.trajectory("BC-7", 0);
        AutoTrajectory C7toSource = routine.trajectory("BC-7", 1);
        AutoTrajectory toC8 = routine.trajectory("BC-8", 0);
        AutoTrajectory C8toSource = routine.trajectory("BC-8", 1);
        AutoTrajectory toC9 = routine.trajectory("BC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("BC-9", 1);
        AutoTrajectory toC10 = routine.trajectory("BC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("BC-10", 1);

        LogUtil.recordPose2d("autostart", startToC5.getInitialPose().get());

        routine.active().onTrue(
                Commands.sequence(
                        startToC5.resetOdometry(),
                        startToC5.cmd()
                )
        );

        startToC5.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        startToC5.done().onTrue(
                scoreL4WaitLower(C5toSource.cmd(), SCORING_DELAY)
        );

        C5toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC7.cmd())
        );

        toC7.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC7.done().onTrue(
                scoreL4WaitLower(C7toSource.cmd(), SCORING_DELAY)
        );

        C7toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC8.cmd())
        );

        toC8.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC8.done().onTrue(
                scoreL4WaitLower(C8toSource.cmd(), SCORING_DELAY)
        );

        C8toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC9.cmd())
        );

        toC9.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC9.done().onTrue(
                scoreL4WaitLower(C9toSource.cmd(), SCORING_DELAY)
        );

        C9toSource.done().onTrue(
                superstructure.waitForIntake().andThen(toC10.cmd())
        );

        toC10.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
                elevator.L4()
        );

        toC10.done().onTrue(
                scoreL4WaitLower(C10toSource.cmd(), SCORING_DELAY)
        );

        return routine;
    }

    private void attachSegment(AutoTrajectory travel, AutoTrajectory source, int branch, AutoTrajectory nextTravel) {
        travel.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(elevator.L4());
        travel.done().onTrue(scoreL4AutoWaitLower(source.cmd(), SCORING_DELAY, branch));
        if (nextTravel != null) {
            source.done().onTrue(superstructure.waitForIntake().andThen(nextTravel.cmd()));
        }
    }

    public AutoRoutine getFiveCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CTA");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(9, 4), swerve),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(10, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(11, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(0, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(1, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait()
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CBA");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(4, 4), swerve),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(3, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(2, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(1, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait(),
                        superstructure.waitForIntake().andThen(AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(0, 4), swerve)),
                        elevator.L0(),
                        alignToSourceAndWait()
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
