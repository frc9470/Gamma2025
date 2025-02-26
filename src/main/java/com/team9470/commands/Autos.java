package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.*;
import com.team9470.util.LogUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Autos {
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

    public Command scoreCoral() {
        return coralManipulator.scoreCommand();
    }

    public AutoRoutine getFourCoralTest() {
        AutoRoutine routine = autoFactory.newRoutine("4C Test");

        // Trajectories
        AutoTrajectory toC9 = routine.trajectory("TC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("TC-9", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);
        AutoTrajectory toC12 = routine.trajectory("TC-12", 0);
        AutoTrajectory C12toSource = routine.trajectory("TC-12", 1);

        routine.active().onTrue(
            Commands.sequence(
                    toC9.resetOdometry(),
                    toC9.cmd()
            )
        );

        toC9.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
            elevator.L4()
        );

        toC10.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
            elevator.L4()
        );

        toC11.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
            elevator.L4()
        );

        toC12.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
            elevator.L4()
        );

        toC9.done().onTrue(
            scoreL4WaitLower(C9toSource.cmd(), SCORING_DELAY)
        );

        toC10.done().onTrue(
            scoreL4WaitLower(C10toSource.cmd(), SCORING_DELAY)
        );

        toC11.done().onTrue(
            scoreL4WaitLower(C11toSource.cmd(), SCORING_DELAY)
        );

        toC12.done().onTrue(
            scoreL4WaitLower(C12toSource.cmd(), SCORING_DELAY)
        );

        C9toSource.done().onTrue(
                toC10.cmd()
        );

        C10toSource.done().onTrue(
                toC11.cmd()
        );

        C11toSource.done().onTrue(
                toC12.cmd()
        );

        return routine;
    }

    public AutoRoutine getThreeCoralTest() {
        AutoRoutine routine = autoFactory.newRoutine("3C Test");

        // Trajectories
        AutoTrajectory startC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1");

        AutoTrajectory toC9 = routine.trajectory("TC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("TC-9", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);

        routine.active().onTrue(
            Commands.sequence(
                    toC9.resetOdometry(),
                    toC9.cmd()
            )
        );

        toC9.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
            elevator.L4()
        );

        toC10.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
            elevator.L4()
        );

        toC11.atTimeBeforeEnd(ELEVATOR_DELAY).onTrue(
            elevator.L4()
        );

        toC9.done().onTrue(
            scoreL4WaitLower(C9toSource.cmd(), SCORING_DELAY)
        );

        toC10.done().onTrue(
            scoreL4WaitLower(C10toSource.cmd(), SCORING_DELAY)
        );

        toC11.done().onTrue(
            scoreL4WaitLower(C11toSource.cmd(), SCORING_DELAY)
        );

        C9toSource.done().onTrue(
                toC10.cmd()
        );

        C10toSource.done().onTrue(
                toC11.cmd()
        );

        return routine;
    }

    public AutoRoutine getTwoCoralTest() {
        AutoRoutine routine = autoFactory.newRoutine("2C Test");

        // Trajectories
        AutoTrajectory toC9 = routine.trajectory("TC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("TC-9", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);

        routine.active().onTrue(
            Commands.sequence(
                    toC9.resetOdometry(),
                    toC9.cmd()
            )
        );

        toC9.done().onTrue(
                scoreL4WaitLower(C9toSource.cmd(), 0.5)
        );

        C9toSource.done().onTrue(
                toC10.cmd()
        );

        toC10.done().onTrue(
                scoreL4WaitLower(C10toSource.cmd(), 0.5)
        );

        return routine;
    }

    public AutoRoutine getBottomThreeCoralTest() {
        AutoRoutine routine = autoFactory.newRoutine("B3C Test");

        // Trajectories
        AutoTrajectory startToC5 = routine.trajectory("S-5");
        AutoTrajectory C5toSource = routine.trajectory("BC-5", 1);
        AutoTrajectory toC7 = routine.trajectory("BC-7", 0);
        AutoTrajectory C7toSource = routine.trajectory("BC-7", 1);
        AutoTrajectory toC8 = routine.trajectory("BC-8", 0);
        AutoTrajectory C8toSource = routine.trajectory("BC-8", 1);

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

        return routine;
    }

    public AutoRoutine getThreeCoralTop() {
        AutoRoutine routine = autoFactory.newRoutine("3CT Test");


        // Trajectories
        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1", 1);
        AutoTrajectory toC12 = routine.trajectory("TC-12", 0);
        AutoTrajectory C12toSource = routine.trajectory("TC-12", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);

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

        toC10.done().onTrue(
            scoreL4()
        );

        return routine;
    }

    public AutoRoutine getThreeCoralTopAuto() {
        AutoRoutine routine = autoFactory.newRoutine("3CTA Test");


        // Trajectories
        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1", 1);
        AutoTrajectory toC12 = routine.trajectory("TC-12", 0);
        AutoTrajectory C12toSource = routine.trajectory("TC-12", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);

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

        toC10.done().onTrue(
                scoreL4AutoWaitLower(new InstantCommand(), SCORING_DELAY, 12)
        );

        return routine;
    }

}
