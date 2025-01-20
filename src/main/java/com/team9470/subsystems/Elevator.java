package com.team9470.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Ports;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.ElevatorConstants;
import static com.team9470.Constants.ElevatorConstants.DIST_PER_ROTATION;
import static edu.wpi.first.units.Units.*;

/**
 * Example Elevator subsystem that:
 * 1) Stores position/velocity in meters.
 * 2) Has a simple homing state machine.
 * 3) Logs telemetry to SmartDashboard.
 */
public class Elevator extends SubsystemBase {

    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotorFollower;

    // Control objects
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final VoltageOut homingVoltage = new VoltageOut(ElevatorConstants.HOMING_OUTPUT);

    // Conversion factor: how many rotations per meter?
    // distancePerRotation ~ 0.2794 m, so ~3.58 rotations per meter
    private final double rotationsPerMeter =
            1.0 / DIST_PER_ROTATION.in(Meters);

    // Homing logic
    private boolean needsHoming = true;
    private HomingState homingState = HomingState.IDLE;

    // Our "goal" position in meters
    private Distance targetPosition = Meters.of(0);

    // Keep track of time when homing started
    private Time homingStartTime = Seconds.of(0);

    // PeriodicIO for reading/writing
    private final PeriodicIO periodicIO = new PeriodicIO();

    private enum HomingState {
        IDLE,       // Not homing
        HOMING,     // Running motor downward
        HOMED       // We found bottom and zeroed
    }

    /**
     * Container for inputs and outputs that we want to log.
     * The idea is to gather all I/O in one place.
     */
    public static class PeriodicIO {
        // ---------------- Inputs ----------------
        public Time timestamp;
        public Distance positionMeters;         // Actual elevator position (m)
        public LinearVelocity velocityMps;      // Actual elevator velocity (m/s)
        public Current current;                 // Stator current
        public Distance closedLoopError;        // difference between on-loop SETPOINT & actual

        // ---------------- Outputs / Telemetry ----------------
        public Distance goal;                         // Overall goal of the system
        public Distance desiredPositionMeters;        // The PER-LOOP commanded position in meters
        public double desiredPositionRotations;       // The PER-LOOP commanded position in rotations
        public double desiredVelocityRotPerSec;       // The commanded velocity in rotations/sec
        public LinearVelocity desiredVelocityMps;     // The commanded velocity in m/s
        public HomingState homingState;               // Current homing state
    }

    public Elevator() {
        elevatorMotor = TalonFXFactory.createDefaultTalon(Ports.ELEVATOR_MAIN);
        elevatorMotorFollower = TalonFXFactory.createPermanentFollowerTalon(
                Ports.ELEVATOR_FOLLOWER, Ports.ELEVATOR_MAIN, true);

        TalonUtil.applyAndCheckConfiguration(elevatorMotor, ElevatorConstants.ElevatorFXConfig());
        TalonUtil.applyAndCheckConfiguration(elevatorMotorFollower, ElevatorConstants.ElevatorFXConfigFollower());
    }

    @Override
    public void periodic() {
        // 1) Read inputs
        readPeriodicInputs();

        // 2) Handle state machine logic (homing & normal)
        if (needsHoming) {
            updateHomingLogic();
        } else {
            homingState = HomingState.IDLE;
        }
        // Store the homing state in periodicIO for telemetry
        periodicIO.homingState = homingState;

        // 3) Write outputs (calculate setpoints and command the motor)
        writePeriodicOutputs();

        // 4) Log data to SmartDashboard (or any other telemetry sink)
        logTelemetry();
    }

    // ------------------ Public Methods ------------------

    /** Command the elevator to a given (meter) setpoint. */
    public void setPosition(Distance position) {
        targetPosition = position;
    }

    /** Get the elevator’s current position in meters. */
    public Distance getPosition() {
        return periodicIO.positionMeters;
    }

    /** Return true if we are in the middle of homing. */
    public boolean isHoming() {
        return homingState == HomingState.HOMING;
    }

    // Example: check if we might be stalling
    public boolean isStalling() {
        return periodicIO.current.in(Amps) >= ElevatorConstants.STALL_CURRENT;
    }

    // ------------------ Internal (Periodic) Methods ------------------

    /** Reads sensor data from the motor and updates periodicIO fields. */
    private void readPeriodicInputs() {
        periodicIO.timestamp = Seconds.of(Timer.getFPGATimestamp());

        // The motor’s position is in rotations. Convert to meters.
        double rotations = elevatorMotor.getPosition().getValue().in(Rotations);
        periodicIO.positionMeters = Meters.of(rotations / rotationsPerMeter);

        // The motor’s velocity is in rotations/sec. Convert to m/s.
        double rotPerSec = elevatorMotor.getVelocity().getValue().in(RotationsPerSecond);
        periodicIO.velocityMps = MetersPerSecond.of(rotPerSec / rotationsPerMeter);

        // Current
        periodicIO.current = elevatorMotor.getStatorCurrent().getValue();

        // For closed-loop error (just for debugging),
        // we can compare target vs. actual in meters:
        periodicIO.closedLoopError = DIST_PER_ROTATION.times(elevatorMotor.getClosedLoopError().getValue());

        // For telemetry
        periodicIO.goal = targetPosition;
        periodicIO.desiredPositionMeters = DIST_PER_ROTATION.times(elevatorMotor.getClosedLoopReference().getValue());
        periodicIO.desiredPositionRotations = elevatorMotor.getClosedLoopReference().getValue();
        periodicIO.desiredVelocityRotPerSec = elevatorMotor.getClosedLoopReference().getValue();
        periodicIO.desiredVelocityMps = DIST_PER_ROTATION.times(elevatorMotor.getClosedLoopReference().getValue()).per(Seconds);
    }

    /** Runs the homing state machine if needed. */
    private void updateHomingLogic() {
        switch (homingState) {
            case IDLE:
                // Example condition: only start homing if the elevator
                // is basically at the commanded position (meaning not in use).
                if (targetPosition.isNear(periodicIO.positionMeters, Meters.of(0.01))) {
                    homingState = HomingState.HOMING;
                    homingStartTime = periodicIO.timestamp;
                }
                break;

            case HOMING:
                // 1) If velocity is near zero or current is high, we assume bottom
                // 2) Or we time out
                boolean velocityStalled =
                        Math.abs(periodicIO.velocityMps.in(MetersPerSecond)) < 0.01;
                boolean currentTooHigh = isStalling();
                boolean timeOut = periodicIO.timestamp
                        .minus(homingStartTime)
                        .gt(ElevatorConstants.HOMING_TIMEOUT);

                if (velocityStalled || currentTooHigh || timeOut) {
                    // We consider ourselves at the bottom => zero the sensor
                    elevatorMotor.setPosition(0);
                    homingState = HomingState.HOMED;
                }
                break;

            case HOMED:
                // Done homing. Turn off the need for homing, remain in IDLE next loop.
                needsHoming = false;
                homingState = HomingState.IDLE;
                break;
        }
    }

    /** Write outputs back to the motor(s). */
    private void writePeriodicOutputs() {
        if (homingState == HomingState.HOMING) {
            // Force a negative voltage to push elevator down
            elevatorMotor.setControl(homingVoltage);

            // For telemetry: set the "desired position/velocity" to something informative
            periodicIO.desiredPositionMeters = Meters.of(0);
            periodicIO.desiredPositionRotations = 0;
            periodicIO.desiredVelocityRotPerSec = 0;
            periodicIO.desiredVelocityMps = MetersPerSecond.of(0);
        } else {
            // Normal control
            // Convert target position (meters) to rotations:
            double targetRotations = targetPosition.in(Meters) * rotationsPerMeter;

            // Let DynamicMotionMagicVoltage handle the motion magic.
            // We set velocity + accel so it knows our motion constraints.
            elevatorMotor.setControl(
                    motionMagic
                            .withPosition(targetRotations)
                            .withSlot(0)
                            .withEnableFOC(true)
            );
        }

        // Once we think we’ve reached the setpoint, request a re-home
        // so next time we move from a known zero.
        // Or do it only if the setpoint > 0, etc.
        if (!needsHoming &&
                (homingState != HomingState.HOMING) &&
                periodicIO.closedLoopError.abs(Meters) < 0.01 &&
                Math.abs(periodicIO.velocityMps.in(MetersPerSecond)) < 0.01) {
            needsHoming = true;
        }
    }

    /** Publish key telemetry values to SmartDashboard. */
    private void logTelemetry() {
        // Actual sensor readings
        SmartDashboard.putNumber("Elevator/Position_m", periodicIO.positionMeters.in(Meters));
        SmartDashboard.putNumber("Elevator/Velocity_mps", periodicIO.velocityMps.in(MetersPerSecond));
        SmartDashboard.putNumber("Elevator/Current_A", periodicIO.current.in(Amps));
        SmartDashboard.putNumber("Elevator/Error_m", periodicIO.closedLoopError.in(Meters));

        // Goal
        SmartDashboard.putNumber("Elevator/Goal", periodicIO.goal.in(Meters));

        // Desired setpoints
        SmartDashboard.putNumber("Elevator/DesiredPosition_m", periodicIO.desiredPositionMeters.in(Meters));
        SmartDashboard.putNumber("Elevator/DesiredPosition_rot", periodicIO.desiredPositionRotations);
        SmartDashboard.putNumber("Elevator/DesiredVelocity_rps", periodicIO.desiredVelocityRotPerSec);
        SmartDashboard.putNumber("Elevator/DesiredVelocity_mps", periodicIO.desiredVelocityMps.in(MetersPerSecond));

        // Homing info
        SmartDashboard.putString("Elevator/HomingState", periodicIO.homingState.toString());
        SmartDashboard.putBoolean("Elevator/NeedsHoming", needsHoming);
    }

    // ------------------ External (Command) Methods ------------------
    public Command getHomingCommand() {
        return new Command() {
            @Override
            public void execute() {
                needsHoming = true;
            }

            @Override
            public boolean isFinished() {
                return homingState == HomingState.HOMED;
            }
        };
    }

    public Command getMoveToPositionCommand(Distance position) {
        return new Command() {
            @Override
            public void execute() {
                setPosition(position);
            }

            @Override
            public void end(boolean interrupted) {
                setPosition(Meters.of(0));
            }

            @Override
            public boolean isFinished() {
                return getPosition().isNear(position, Meters.of(0.01));
            }
        };
    }

    /* command that doesn't wait for the elevator to reach the setpoint */
    public Command getMoveToPositionCommandNoWait(Distance position) {
        return new Command() {
            @Override
            public void execute() {
                setPosition(position);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Command L1(){
        return getMoveToPositionCommand(ElevatorConstants.L1);
    }

    public Command L2(){
        return getMoveToPositionCommand(ElevatorConstants.L2);
    }

    public Command L3(){
        return getMoveToPositionCommand(ElevatorConstants.L3);
    }

    public Command L4(){
        return getMoveToPositionCommand(ElevatorConstants.L4);
    }

    public Command intake(){
        return getMoveToPositionCommand(ElevatorConstants.INTAKE);
    }


}