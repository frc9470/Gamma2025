package com.team9470;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import static edu.wpi.first.units.Units.*;

public final class Constants {

    public static final class ElevatorConstants {
        // Physical geometry
        // For example: 22 * 0.25 inches * 2 = 11 inches per rotation
        // Convert to meters for internal usage
        public static final Distance DIST_PER_ROTATION =
                Units.Inches.of(22 * 0.25).times(2);
        public static final double rotationsPerMeter = 1.0 / DIST_PER_ROTATION.in(Meters);

        // Gains
        public static final double kP = 1.0;
        public static final double kG = 0.18;
        public static final double kV = 5.33;
        // etc...

        // Motion config
        public static final LinearVelocity CRUISE_VELOCITY = Units.MetersPerSecond.of(5.0);
        public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(120.0);
        public static final double JERK = 10.0;

        // Homing
        public static final Voltage HOMING_OUTPUT = Units.Volts.of(-2.0);
        public static final LinearVelocity HOMING_MAX_VELOCITY = Units.MetersPerSecond.of(0.1);
        public static final Distance HOMING_ZONE = Meters.of(0.1);
        public static final Time HOMING_TIMEOUT = Units.Seconds.of(0.5);

        // Current limits
        public static final double STALL_CURRENT = 40; // example

        public static final Distance L1 = Meters.of(0);
        public static final Distance L2 = Meters.of(.5);
        public static final Distance L3 = Meters.of(1);
        public static final Distance L4 = Meters.of(1.5);
        public static final Distance INTAKE = Meters.of(0);


        public static TalonFXConfiguration ElevatorFXConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(MetersPerSecond) * rotationsPerMeter;; // We'll override in code with setControl
            config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(MetersPerSecondPerSecond) * rotationsPerMeter;;
            config.MotionMagic.MotionMagicJerk = JERK;
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            config.Slot0.kV = kV;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = kP;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kG = kG;
            config.Slot0.kS = 0.0;
            config.Feedback.SensorToMechanismRatio = 6.0;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STALL_CURRENT;
            return config;
        }

        public static TalonFXConfiguration ElevatorFXConfigFollower(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STALL_CURRENT;
            return config;
        }
    }

    public static final class AlgaeConstants{
        public static final double KS = 0;
        public static final double KG = 0;
        public static final double KV = 0;
        public static final double CORAL_THRESHOLD = Math.toRadians(45);
        public static final double ALGAE_IN_THRESHOLD = 0;
        public static final double ANGLE_UP = Math.toRadians(45);
        public static final double ANGLE_DOWN = Math.toRadians(45);
        public static final int ARM_MOTOR_ID = 0;
        public static final int ROLLER_MOTOR_ID = 1;

        public static TalonFXConfiguration getConfigs(){
            TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

            // set slot 0 gains
            var slot0Configs = talonFXConfigs.Slot0;
            slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
            slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
            slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
            slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
            slot0Configs.kI = 0; // no output for integrated error
            slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

            // set Motion Magic settings
            var motionMagicConfigs = talonFXConfigs.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
            motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
            motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

            return talonFXConfigs;
        }
    }
}