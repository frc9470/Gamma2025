package com.team9470;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public final class Constants {

    // CHANGE THIS BECAUSE THIS IS SAMPLE FROM LAST YR
     public static class VisionConstants {
        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(Units.Inches.of(-6.416074), Units.Inches.of(+6.234908), Units.Inches.of(24.876993),
                new Rotation3d(-0.12384803489944651, -0.3272010156831353, 0.3695356336033198));
        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(Units.Inches.of(-6.416074), Units.Inches.of(-6.234908), Units.Inches.of(24.876993),
                new Rotation3d(0.12384803489944651, -0.3272010156831353, -0.3695356336033198));


    }

    public static final class ElevatorConstants {
        // Physical geometry
        // For example: 22 * 0.25 inches * 2 = 11 inches per rotation
        // Convert to meters for internal usage
        public static final Distance DIST_PER_ROTATION =
                Units.Inches.of(22 * 0.25).times(2);
        public static final double rotationsPerMeter = 1.0 / DIST_PER_ROTATION.in(Meters);
        public static final double GEAR_RATIO = 6.0;
        public static final double MASS = 9.072;
        public static final double DRUM_RADIUS = DIST_PER_ROTATION.in(Meter) / (2 * Math.PI);

        // Gains
        public static final double kP = 7;
        public static final double kG = 0.38;
        public static final double kV = 0.85;
        public static final double kA = 0.01;
        // etc...

        // Motion config
        public static final LinearVelocity CRUISE_VELOCITY = Units.MetersPerSecond.of(2.0);
        public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(10.0);
        public static final double JERK = 0;

        // Homing
        public static final Voltage HOMING_OUTPUT = Units.Volts.of(-2.0);
        public static final LinearVelocity HOMING_MAX_VELOCITY = Units.MetersPerSecond.of(0.1);
        public static final Distance HOMING_ZONE = Meters.of(0.1);
        public static final Time HOMING_TIMEOUT = Units.Seconds.of(0.5);

        // Current limits
        public static final double STALL_CURRENT = 40; // example

        public static final Distance HOME_POSITION = Meters.of(0);
        public static final Distance L1 = Meters.of(0.2);
        public static final Distance L2 = Meters.of(.4);
        public static final Distance L3 = Meters.of(.7);
        public static final Distance L4 = Meters.of(1.37);
        public static final Distance INTAKE = Meters.of(0);


        public static TalonFXConfiguration ElevatorFXConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(MetersPerSecond) * rotationsPerMeter;
            SmartDashboard.putNumber("Elevator/CruiseVelocity", config.MotionMagic.MotionMagicCruiseVelocity);
            config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(MetersPerSecondPerSecond) * rotationsPerMeter;
            config.MotionMagic.MotionMagicJerk = JERK;
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            config.Slot0.kV = kV;
            config.Slot0.kA = kA;
            config.Slot0.kP = kP;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kG = kG;
            config.Slot0.kS = 0.0;
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STALL_CURRENT;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

    public static final class AlgaeConstants {
        public static final double KS = 0;
        public static final double KG = 0;
        public static final double KV = 0;
        public static final double CORAL_THRESHOLD = 75; // Angle in degrees
        public static final double ALGAE_IN_THRESHOLD = 0; // Current
        public static final double ANGLE_UP = 90;
        public static final double ANGLE_DOWN = 0;

        public static final double HOMING_SPEED = -1;
        public static final double HOMING_THRESHOLD = 0; // Current

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

            talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return talonFXConfigs;
        }
    }

    public static final class CoralConstants {
        public static final Voltage TAKE_IN_SPEED = Volts.of(3);
        public static final Voltage COAST_SPEED = Volts.of(2);
        public static final double BREAK_TIMEOUT = .1;
    }

    public static final class DriverAssistConstants {
        public static final Pose2d[] BLUE_REEF_POSITIONS = { // {x (m), y (m), angle (rad)}
            new Pose2d(3.7454309463500977, 5.406795501708984, new Rotation2d(-1.0584074157409784)),
            new Pose2d(2.9004666805267334, 4.025999546051025, new Rotation2d(0)),
            new Pose2d(3.7042129039764404, 2.6452038288116455, new Rotation2d(1.0303770533621297)),
            new Pose2d(5.270488739013672, 2.645203113555908, new Rotation2d(2.1375260206777438)),
            new Pose2d(6.094844341278076, 4.025998592376709, new Rotation2d(3.141592653589793)),
            new Pose2d(5.249879837036133, 5.40679407119751, new Rotation2d(-2.095592098445004)),
        };

        public static final Pose2d[] RED_REEF_POSITIONS = { // {x (m), y (m), angle (rad)}
            new Pose2d(12.339337348937988, 5.406795501708984, new Rotation2d(-1.0584074157409784)),
            new Pose2d(11.514982223510742, 4.025999546051025, new Rotation2d(0)),
            new Pose2d(12.29811954498291, 2.6452038288116455, new Rotation2d(1.0303770533621297)),
            new Pose2d(13.864395141601562, 2.645203113555908, new Rotation2d(2.1375260206777438)),
            new Pose2d(14.647533416748047, 4.025998592376709, new Rotation2d(3.141592653589793)),
            new Pose2d(13.823177337646484, 5.40679407119751, new Rotation2d(-2.095592098445004)),
        };
    }
}