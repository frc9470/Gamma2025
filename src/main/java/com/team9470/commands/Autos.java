package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.AlgaeArm;
import com.team9470.subsystems.CoralManipulator;
import com.team9470.subsystems.Elevator;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Autos(AlgaeArm algaeArm, CoralManipulator coralManipulator, Elevator elevator, Swerve swerve) {
        this.autoFactory = swerve.createAutoFactory();
        this.algaeArm = algaeArm;
        this.coralManipulator = coralManipulator;
        this.elevator = elevator;
        this.swerve = swerve;
    }

    public static final double SCORING_DELAY = 0.3;
    private static final double ELEVATOR_DELAY = 0.5;

    public Command scoreL4(){
        return elevator.L4().andThen(
                coralManipulator.scoreCommand().withTimeout(SCORING_DELAY))
                .andThen(elevator.L0());
    }

    int count = 0;
    public Command scoreL4WaitLower(Command driveAway, double delay){
        return elevator.L4()
                .andThen(new InstantCommand(() -> SmartDashboard.putNumber("Auto L4 Count", ++count)))
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
                    toC9.cmd(),
                    toC10.cmd(),
                    toC11.cmd(),
                    toC12.cmd()
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

        return routine;
    }

    public AutoRoutine getThreeCoralTest() {
        AutoRoutine routine = autoFactory.newRoutine("4C Test");

        // Trajectories
        AutoTrajectory toC9 = routine.trajectory("TC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("TC-9", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);
        AutoTrajectory toC11 = routine.trajectory("TC-11", 0);
        AutoTrajectory C11toSource = routine.trajectory("TC-11", 1);

        routine.active().onTrue(
            Commands.sequence(
                    toC9.resetOdometry(),
                    toC9.cmd(),
                    toC10.cmd(),
                    toC11.cmd()
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
}
