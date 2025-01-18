package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * CoralManipulator - Pranesh
 * Subsystem Requirements:
 * - move wheels forward or backward with command
 * - use a sensor to detect if a coral is in the manipulator
 * - one motor
 */
public class CoralManipulator extends SubsystemBase {
    private TalonFX motor = new TalonFX(0);

    public StatusSignal<Angle> getEncoderPosition(){
        return motor.getPosition();
    }
}
