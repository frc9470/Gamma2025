package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.AlgaeArm;
import com.team9470.subsystems.CoralManipulator;
import com.team9470.subsystems.Elevator;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

    public Command scoreL4(){
        return elevator.L4().andThen(
                coralManipulator.scoreCommand())
                .andThen(elevator.L0());
    }

    public Command scoreCoral() {
        return coralManipulator.scoreCommand()
    }

    public AutoRoutine getFourCoralTest(){
        AutoRoutine routine = autoFactory.newRoutine("4C Test");

        // Trajectories
        AutoTrajectory startToC1 = routine.trajectory("S-1");
        AutoTrajectory C1toSource = routine.trajectory("TC-1", 1);
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
                        startToC1.resetOdometry(),
                        startToC1.cmd()
                )
        );

        startToC1.done().onTrue(
                scoreL4().andThen(C1toSource.cmd())
        );

        C1toSource.done().onTrue(toC12.cmd());
        toC12.done().onTrue(scoreL4().andThen(C12toSource.cmd()));

        C12toSource.done().onTrue(toC11.cmd());
        toC11.done().onTrue(scoreL4().andThen(C11toSource.cmd()));

        C11toSource.done().onTrue(toC10.cmd());
        toC10.done().onTrue(scoreL4().andThen(C10toSource.cmd()));

        C10toSource.done().onTrue(toC9.cmd());
        toC9.done().onTrue(scoreL4().andThen(C9toSource.cmd()));

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
                    toC9.cmd(),
                    C9toSource.cmd(),
                    toC10.cmd(),
                    C10toSource.cmd()
            )
        );

        toC9.done().onTrue(
                scoreL4().andThen(C9toSource.cmd())
        );

        toC10.done().onTrue(
                scoreL4().andThen(C10toSource.cmd())
        );

        return routine;
    }

    public AutoRoutine getTwoCoralOptimizedTest() {
        AutoRoutine routine = autoFactory.newRoutine("2C Test");

        // Trajectories
        AutoTrajectory toC9 = routine.trajectory("TC-9", 0);
        AutoTrajectory C9toSource = routine.trajectory("TC-9", 1);
        AutoTrajectory toC10 = routine.trajectory("TC-10", 0);
        AutoTrajectory C10toSource = routine.trajectory("TC-10", 1);

        routine.active().onTrue(
            Commands.sequence(
                    toC9.resetOdometry(),
                    Commands.parallel(
                        toC9.cmd(),
                        elevator.L4()
                    ),
                    Commands.parallel(
                        C9toSource.cmd(),
                        elevator.L0()
                    ),
                    Commands.parallel(
                        toC10.cmd(),
                        elevator.L4()
                    ),
                    Commands.parallel(
                        C10toSource.cmd(),
                        elevator.L0()
                    )
            )
        );

        toC9.done().onTrue(
                scoreCoral()
        );

        toC10.done().onTrue(
                scoreCoral()
        );

        return routine;
    }


}
