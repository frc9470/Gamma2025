package com.team9470.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.math.controller.ArmFeedforward;

import com.team9470.Constants.AlgaeConstants;
import com.team9470.Ports;

/**
 * AlgaeArm - Neev + Aadit
 * Subsystem Requirements:
 * - move arm up or down with command - DONE
 * - use a sensor to detect if the arm is at the top or bottom - DONE
 * - detect if the arm is holding algae with current sensor - DONE
 * - detect if the arm is in the way of the CoralManipulator - DONE
 * - roll algae wheels to manipulate algae - DONE
 * - one arm motor, one wheel motor - DONE
 * - motion magicc - DONE
 */
public class AlgaeArm extends SubsystemBase {
    private double setpoint = 0;
    private int onTop = 0; // top = 1; bottom = -1
    private boolean homing = false;
    private TalonFX motor = TalonFXFactory.createDefaultTalon(Ports.ALGAE_PIVOT);
    private TalonFX rollers = TalonFXFactory.createDefaultTalon(Ports.ALGAE_ROLLER);
    // private ArmFeedforward ff = new ArmFeedforward(AlgaeConstants.KS, AlgaeConstants.KG, AlgaeConstants.KV);
    final MotionMagicVoltage request = new MotionMagicVoltage(0);

    public AlgaeArm(CoralManipulator coral){
        motor.getConfigurator().apply(AlgaeConstants.getConfigs());
    }

    public void periodic(){
        if(homing == true){
            motor.set(AlgaeConstants.HOMING_SPEED);
            if(getPivotCurrent().gte(Units.Amps.of(AlgaeConstants.HOMING_THRESHOLD))){
                homing = false;
                motor.setPosition(0);
                motor.set(0);
            }
            return;
        }
        motor.setControl(request.withPosition(Units.Degrees.of(setpoint)));
        // motor.set(ff.calculate(setpoint, 0));
    }

    public void armUp(){
        setpoint = AlgaeConstants.ANGLE_UP;
        onTop = 1;
    }

    public void armDown(){
        setpoint = AlgaeConstants.ANGLE_DOWN;
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

    public Current getPivotCurrent(){
        return motor.getStatorCurrent().getValue();
    }

    public Current getRollerCurrent(){
        return rollers.getStatorCurrent().getValue();
    }

    public boolean blockingCoralManipulator(){
        return getAngle().gte(Units.Degrees.of(AlgaeConstants.CORAL_THRESHOLD));
    }

    public boolean algaeIn(){
        return getRollerCurrent().gte(Units.Amps.of(AlgaeConstants.ALGAE_IN_THRESHOLD));
    }

    public void setHoming(){
        homing = true;
    }
}
