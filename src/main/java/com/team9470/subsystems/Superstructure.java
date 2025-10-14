package com.team9470.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    private final Elevator elevator;
    private final Indexer indexer;
    private final Arm arm;
    private final Intake intake;

    public Superstructure(Mechanism2d mech) {
        this.elevator = new Elevator(mech);
        this.indexer = new Indexer();
        this.arm = new Arm();
        this.intake = new Intake();
    }

    public Command reverseCoral() {
        return indexer.reverseCommand();
    }

    // Stow algae in alternate position (stow location 2) for coral scoring:
    // Raise elevator to L2 and move algae below the coral manipulator.
    public Command algaeUp() {
        return Commands.none();
    }

    public Command algaeDown() {
        return Commands.none();
    }

    public Command raise(int level){
        return elevator.getLevelCommand(level);
    }

    public Command raise(Supplier<Integer> getLevel){
        return elevator.getLevelCommand(getLevel.get());
    }

    
    public Command score() {
        return indexer.outputCommand();
    }

    public Command scoreSlow() {
        return indexer.outputCommand();
    }

    // Trigger the algae arm’s homing routine.
    public Command triggerAlgaeHoming() {
        return Commands.none();
    }

    public Command waitForIntake() {
        return new WaitUntilCommand(indexer::hasCoral);
    }

    public Command funnelOut() {
        return Commands.none();
    }

    public Command climberAction() {
        return Commands.none();
    }

    public boolean hasGamePiece() {
        return indexer.hasCoral();
    }

    // Accessors for the individual subsystems.
    public Elevator getElevator() {
        return elevator;
    }

    public Indexer getIndexer() {
        return indexer;
    }

    public Arm getArm() {
        return arm;
    }

    public Intake getIntake() {
        return intake;
    }

    public Command scoreAndFunnel() {
        return indexer.outputCommand();
    }

    // -- Scoring Commands (only elevator and arm) --

    // Coral Scoring Commands
    public Command scoreCoralL4() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        elevator.L4(),
                        arm.coralL4BeforeScoringCommand()
                ),
                arm.coralL4ScoringCommand()
        );
    }

    public Command scoreCoralL3() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        elevator.L3(),
                        arm.coralL3BeforeScoringCommand()
                ),
                arm.coralL3ScoringCommand()
        );
    }

    public Command scoreCoralL2() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        elevator.L2(),
                        arm.coralL2BeforeScoringCommand()
                ),
                arm.coralL2ScoringCommand()
        );
    }

    public Command scoreCoralL1() {
        return new SequentialCommandGroup(
                elevator.L1(),
                arm.coralL1ScoringCommand()
        );
    }

    // Algae Scoring Commands
    public Command scoreAlgaeBarge() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        elevator.L4(),
                        arm.algaeBargeBeforeScoringCommand()
                ),
                arm.algaeBargeScoringCommand()
        );
    }

    public Command scoreAlgaeProcessor() {
        return new SequentialCommandGroup(
                elevator.L0(),
                arm.algaeProcessorScoringCommand()
        );
    }

    // Intake Commands
//    public Command intakeCoral() {
//        return new ParallelCommandGroup(
//                elevator.intake(), // Move to intake height
//                arm.coralIntakeCommand()
//        );
//    }
    public Command intakeCoralGround() {
        return new ParallelCommandGroup(
                intake.goDownAndStartRollersCommand(),
                indexer.intakeCommand()
        );
    }

    public Command releaseCoralGround() {
        return intake.retractAndStopRollersCommand();
    }

    public Command intakeAlgaeGround() {
        return new ParallelCommandGroup(
                elevator.intake(), // Move to intake height
                arm.algaeGroundIntakeCommand()
        );
    }

    public Command intakeAlgaeReef() {
        return new ParallelCommandGroup(
                elevator.intake(), // Move to intake height
                arm.algaeReefIntakeCommand()
        );
    }
}