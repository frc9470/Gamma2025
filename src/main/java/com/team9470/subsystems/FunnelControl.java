package com.team9470.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team9470.Constants;
import com.team9470.Ports;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelControl extends SubsystemBase {
    private final TalonFX controlMotor = TalonFXFactory.createDefaultTalon(Ports.FUNNEL_CONTROL);

    public FunnelControl(){
        controlMotor.getConfigurator().apply(Constants.FunnelControlConstants.getMotorConfig());
        setDefaultCommand(coastUnless());
    }

    public Command coastUnless() {
        return run(() -> controlMotor.setVoltage(0));
    }

    public Command runOut() {
        return run(() -> controlMotor.setVoltage(3));
    }
}
