package com.team9470.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.math.controller.ArmFeedforward;

/**
 * AlgaeArm - Neev + Aadit
 * Subsystem Requirements:
 * - move arm up or down with command - DONE
 * - use a sensor to detect if the arm is at the top or bottom - DONE
 * - detect if the arm is holding algae with current sensor - DONE
 * - detect if the arm is in the way of the CoralManipulator - DONE
 * - roll algae wheels to manipulate algae - DONE
 * - one arm motor, one wheel motor - DONE
 * - motion magicc - IDK
 */
public class AlgaeArm extends SubsystemBase {
    // constants (random values for now)
    final double KS = 0;
    final double KG = 0;
    final double KV = 0;
    final double CORAL_THRESHOLD = Math.toRadians(45);
    final double ALGAE_IN_THRESHOLD = 0;
    final double ANGLE_UP = Math.toRadians(45);
    final double ANGLE_DOWN = Math.toRadians(45);
    final int ARM_MOTOR_ID = 0;
    final int ROLLER_MOTOR_ID = 1;


    private double setpoint = 0;
    private int onTop = 0; // top = 1; bottom = -1
    private TalonFX motor = new TalonFX(ARM_MOTOR_ID);
    private TalonFX rollers = new TalonFX(ROLLER_MOTOR_ID);
    private ArmFeedforward ff = new ArmFeedforward(KS, KG, KV);

    public AlgaeArm(CoralManipulator coral){
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

    public Angle getAngle(){
        return motor.getPosition().getValue();
    }
    public Current getRollerCurrent(){
        return rollers.getStatorCurrent().getValue();
    }

    public boolean blockingCoralManipulator(){
        return getAngle().gte(Units.Degrees.of(CORAL_THRESHOLD));
    }

    public boolean AlgaeIn(){
        return getRollerCurrent().gte(Units.Amps.of(ALGAE_IN_THRESHOLD));
    }
}
