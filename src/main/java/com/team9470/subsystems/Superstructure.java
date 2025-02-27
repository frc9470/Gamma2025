package com.team9470.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure extends SubsystemBase {
    private final Elevator elevator;
    private final CoralManipulator coral;
    private final AlgaeArm algae;
    private final LEDs leds;



    public Superstructure(Mechanism2d mech) {
        this.elevator = new Elevator(mech);
        this.coral = new CoralManipulator();
        this.algae = new AlgaeArm(elevator.getElevatorLigament());
        this.leds = LEDs.getInstance();
    }

    // Returns a command to reverse the coral manipulator.
    public Command reverseCoral() {
        return coral.reverseCommand();
    }

    /*
     * Algae Bar Management:
     *
     * - Initial state: algae bar is stored upwards.
     *
     * - Ground Intake:
     *     Deploy the algae arm and spin intake to acquire algae.
     *
     * - Processor Scoring:
     *     Deploy the arm outwards and reverse the spin to push the bar down.
     *     (Optionally, chain an auto–drive command to back away.)
     *
     * - Dealgify (removing algae):
     *     Automatically determine elevator level (L2 or L3) based on reef.
     *     Move elevator to target level then deploy and reverse spin the arm.
     *
     * - Stow Positions (when algae is obtained):
     *     • Default stow (location 1): stores algae between bumper and bar,
     *       lowering the CG but blocking coral scoring.
     *     • Alternate stow (location 2): uses elevator at L2 with arm down,
     *       allowing coral scoring despite a higher CG.
     *
     * - If algae is not obtained, simply stow the bar upwards.
     */

    // Ground intake: deploy and spin to acquire algae.
    public Command groundIntake() {
        return algae.groundDeploy().alongWith(algae.reverse());
    }

    // Processor scoring: deploy outwards and reverse spin.
    // Optionally, you might chain an auto–drive command (e.g. to back away) here.
    public Command processorScore(/*Swerve drivetrain*/) {
        // Example with drivetrain (uncomment if drivetrain command available):
        // return algae.deploy().alongWith(algae.reverse())
        //         .andThen(new InstantCommand(() -> drivetrain.driveBackward()));
        return algae.groundDeploy().alongWith(algae.spin());
    }

    // Return the algae bar if algae is not obtained (stow bar upwards).
    public Command algaeReturn() {
        return algae.stow();
    }

    // Dealgify: move the elevator to the appropriate level (based on reef) and then
    // deploy the arm (with reverse spin) to remove algae.
    public Command dealgify(int level) {
        Command moveElevator = (level == 2) ? elevator.algaeL2() : elevator.algaeL3();
        return moveElevator.alongWith(algae.deploy().alongWith(algae.spin()));
    }


    // Stow algae in default position (stow location 1).
    public Command stowAlgaeDefault() {
        // Assumes that algae.stowDefault() moves the algae into the bumper/bar region.
        return algae.deploy();
    }

    // Stow algae in alternate position (stow location 2) for coral scoring:
    // Raise elevator to L2 and move algae below the coral manipulator.
    public Command raiseAndStow(int level) {
        // Assumes that algae.stowAlternate() is implemented to position the algae for coral scoring.
        return elevator.getLevelCommand(level).alongWith(algae.stowDown());
    }

    public Command raise(int level){
        return elevator.getLevelCommand(level);
    }
    
    public Command score() {
        return coral.scoreCommand();
    }

    // Trigger the algae arm’s homing routine.
    public Command triggerAlgaeHoming() {
        return new InstantCommand(algae::triggerHoming);
    }

    public Command waitForIntake() {
        return new WaitUntilCommand(coral::hasCoral);
    }


    // Accessors for the individual subsystems.
    public Elevator getElevator() {
        return elevator;
    }

    public CoralManipulator getCoral() {
        return coral;
    }

    public AlgaeArm getAlgae() {
        return algae;
    }

    public LEDs getLEDs() {
        return leds;
    }

}