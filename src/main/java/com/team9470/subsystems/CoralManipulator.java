package com.team9470.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.team254.lib.drivers.TalonFXFactory;
import com.team9470.Constants.CoralConstants;
import com.team9470.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;


/**
 * CoralManipulator - Pranesh
 * Subsystem Requirements:
 * - move wheels forward or backward with command
 * - use a sensor to detect if a coral is in the manipulator
 * - one motor
 */
public class CoralManipulator extends SubsystemBase {
    private final TalonFX coralMotor = TalonFXFactory.createDefaultTalon(Ports.CORAL_INTAKE);
    private final DigitalInput coralSensor = new DigitalInput(Ports.CORAL_BREAK);

    public CoralManipulator() {}

    // TODO: verify the beambreak code works correctly
    /**
     * The beambreak is true if the beambreak is NOT "broken" (i.e. something is NOT detected)
     * The beambreak returns false if the beambreak IS "broken" (i.e. something IS detected)
     *
     * @return Whether coral is currently in the intake or not
     */
    public boolean hasCoral() {
        return !coralSensor.get();
    }

    public void brake() {
        coralMotor.set(0);
    }

    public void intake() {
        coralMotor.set(CoralConstants.TAKE_IN_SPEED);
    }

    public void outtake() {
        coralMotor.set(-CoralConstants.TAKE_IN_SPEED);
    }

    public void coast() {
        coralMotor.set(CoralConstants.COAST_SPEED);
    }

    // ---

    public Command outtakeCommand() {
        return this.runEnd(
                this::outtake
                , this::brake
        );
    }

    public Command defaultCommand(boolean wantsIntake) {
        CoralManipulator self = this;

        return new Command() {
           @Override
           public void execute() {
               if (wantsIntake) {
                   intake();
               } else if (!hasCoral()) {
                   coast();
               } else {
                   brake();
               }
           }

           @Override
           public void end(boolean interrupted) {
               brake();
           }

           @Override
           public Set<Subsystem> getRequirements() {
               return Set.of(self);
           }
       };
    }

    public void initDefaultCommand() {
        setDefaultCommand(defaultCommand(false));
    }
}
