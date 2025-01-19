package com.team9470.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.ArmFeedforward;

/**
 * AlgaeArm - Neev + Aadit
 * Subsystem Requirements:
 * - move arm up or down with command - DONE
 * - use a sensor to detect if the arm is at the top or bottom - KINDA DONE
 * - detect if the arm is holding algae with current sensor - IDK
 * - detect if the arm is in the way of the CoralManipulator - KINDA DONE
 * - roll algae wheels to manipulate algae - DONE
 * - one arm motor, one wheel motor - DONE
 * - motion magicc - IDK
 */
public class AlgaeArm extends SubsystemBase {
    final double KS = 0;
    final double KG = 0;
    final double KV = 0;

    final double CORAL_MIN_POS = Math.toRadians(-45);
    final CoralManipulator coral;

    final double ANGLE_UP = Math.toRadians(45);
    final double ANGLE_DOWN = Math.toRadians(45);


    private double setpoint = 0;
    private int onTop = 0; // top = 1; bottom = -1
    private TalonFX motor = new TalonFX(0);
    private TalonFX rollers = new TalonFX(1);
    private ArmFeedforward ff = new ArmFeedforward(KS, KG, KV);

    public AlgaeArm(CoralManipulator coral){
        this.coral = coral;
    }

    public void periodic(){
        ff.calculate(setpoint, 0);
    }

    public void armUp(){
        setpoint = ANGLE_UP;
        onTop = 1;
    }

    public void armDown(){
        setpoint = ANGLE_DOWN;
        onTop = -1;
    }

    public int getArmPos(){
        return onTop;
    }

    public void setRollerSpeed(double speed){
        rollers.set(speed);
    }

    public StatusSignal<Angle> getEncoderPosition(){
        return motor.getPosition();
    }

    public boolean detectCoralManipulator(){
        if(coral.getEncoderPosition().getValueAsDouble() <= CORAL_MIN_POS){
            return true;
        }
        return false;
    }
}
