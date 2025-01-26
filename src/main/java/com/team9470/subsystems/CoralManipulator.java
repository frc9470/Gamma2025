package com.team9470.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team9470.Constants.CoralConstants;
import com.team9470.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;


/**
 * CoralManipulator - Pranesh
 * Subsystem Requirements:
 * - move wheels forward or backward with command
 * - use a sensor to detect if a coral is in the manipulator
 * - one motor
 */
public class CoralManipulator extends SubsystemBase {
    private final TalonFX coralMotor = TalonFXFactory.createDefaultTalon(Ports.CORAL_INTAKE);
    private final DigitalInput coralSensor = new DigitalInput(Ports.CORAL_BREAK);;

    public CoralManipulator() {
        setDefaultCommand(coastUnless());
    }

    @Override
    public void periodic() {


        SmartDashboard.putBoolean("CoralManipulator/HasCoral", hasCoral());

    }

    // it works!
    /**
     * The beambreak is true if the beambreak is NOT "broken" (i.e. something is NOT detected)
     * The beambreak returns false if the beambreak IS "broken" (i.e. something IS detected)
     *
     * @return Whether coral is currently in the intake or not
     */
    public boolean hasCoral() {
        return !coralSensor.get();
    }

    public Command coastUnless(){
        return this.run(() -> {
            if(hasCoral()){
                coralMotor.stopMotor();
            } else {
                coralMotor.setVoltage(CoralConstants.COAST_SPEED.in(Volts));
            }
        });
    }

    public Command scoreCommand(){
        return this.run(() -> coralMotor.setVoltage(CoralConstants.TAKE_IN_SPEED.in(Volts)));
    }

    public Command outtakeCommand() {
        return this.runEnd(
                () -> coralMotor.setVoltage(-CoralConstants.TAKE_IN_SPEED.in(Volts))
                , coralMotor::stopMotor
        );
    }
}
