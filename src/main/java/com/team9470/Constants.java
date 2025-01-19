package com.team9470;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class Constants {
    public final static class ElevatorConstants {
        public static TalonFXConfiguration ElevatorFXConfig(){
            TalonFXConfiguration elevatorMainConfig = new TalonFXConfiguration();
            elevatorMainConfig.MotionMagic.MotionMagicCruiseVelocity = 5.0;
            elevatorMainConfig.MotionMagic.MotionMagicAcceleration = 120.0;
            elevatorMainConfig.MotionMagic.MotionMagicJerk = 10;
            elevatorMainConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            elevatorMainConfig.Slot0.kV = 5.33;
            elevatorMainConfig.Slot0.kA = 0.0;
            elevatorMainConfig.Slot0.kP = 1.0;
            elevatorMainConfig.Slot0.kI = 0.0;
            elevatorMainConfig.Slot0.kD = 0.0;
            elevatorMainConfig.Slot0.kG = 0.18;
            elevatorMainConfig.Slot0.kS = 0.0;
            elevatorMainConfig.Feedback.SensorToMechanismRatio = 6.0;
            elevatorMainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            elevatorMainConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            elevatorMainConfig.CurrentLimits.StatorCurrentLimit = 40;
            return elevatorMainConfig;
        }

        public static TalonFXConfiguration ElevatorFXConfigFollower(){
            TalonFXConfiguration elevatorFollowerConfig = new TalonFXConfiguration();
            elevatorFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            elevatorFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            elevatorFollowerConfig.CurrentLimits.StatorCurrentLimit = 40;
            return elevatorFollowerConfig;
        }

        public static final Voltage HOMING_OUTPUT = Units.Volts.of(-2.0);
        public static final LinearVelocity HOMING_MAX_VELOCITY = Units.MetersPerSecond.of(0.1);
        public static final Distance HOMING_ZONE = Units.Meters.of(0.1);
        public static final Time HOMING_TIMEOUT = Units.Seconds.of(.5);
    }
}
