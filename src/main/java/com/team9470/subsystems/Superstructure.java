package com.team9470.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.Supplier;

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


    // Stow algae in alternate position (stow location 2) for coral scoring:
    // Raise elevator to L2 and move algae below the coral manipulator.
    public Command algaeUp() {
        return algae.up();
    }

    public Command algaeDown() {
        return algae.down();
    }

    public Command raise(int level){
        return elevator.getLevelCommand(level);
    }

    public Command raise(Supplier<Integer> getLevel){
        return elevator.getLevelCommand(getLevel.get());
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