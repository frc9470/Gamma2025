package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Constants.AlgaeConstants;
import com.team9470.Ports;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class AlgaeArm extends SubsystemBase {

    // --- Hardware ---
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;

    // --- Control objects ---
    // Use MotionMagic for smooth motion to a target angle.
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    // When homing, we command a fixed voltage output.
    private final VoltageOut homingVoltage = new VoltageOut(AlgaeConstants.HOMING_OUTPUT);

    // --- Status Signals (for sensor feedback) ---
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Double> setpointSignal;
    private final StatusSignal<Double> setpointVelocitySignal;
    private final StatusSignal<Double> errorSignal;

    // --- Homing state machine ---
    private enum HomingState {
        IDLE,       // Normal operation
        HOMING,     // Running homing (moving toward the limit)
        HOMED       // Homing complete; sensor zeroed
    }
    private boolean needsHoming = true;
    private HomingState homingState = HomingState.IDLE;
    private Time homingStartTime = Seconds.of(0);

    // --- Target angle ---
    private Angle targetAngle = Degrees.of(0);

    // --- Periodic I/O container ---
    public static class PeriodicIO {
        // Inputs
        public Time timestamp;
        public Angle position;                 // Measured pivot angle
        public AngularVelocity velocity;       // Measured angular velocity
        public Current current;                // Pivot motor current
        public Voltage voltage;                // Pivot motor voltage
        public Angle closedLoopError;          // Error (in degrees) between setpoint and actual

        // Outputs / Telemetry
        public Angle goal;                     // Desired target angle
        public Angle setpointAngle;            // Commanded setpoint (from closed-loop)
        public double setpointRaw;             // Raw setpoint value (degrees)
        public double setpointVelocityRaw;     // Raw setpoint velocity (deg/s)
        public AngularVelocity setpointAngularVelocity; // Converted setpoint velocity
        public HomingState homingState;        // Current homing state
    }
    private final PeriodicIO periodicIO = new PeriodicIO();

    public AlgaeArm() {
        // Create the pivot and roller TalonFX instances.
        pivotMotor = TalonFXFactory.createDefaultTalon(Ports.ALGAE_PIVOT);
        rollerMotor = TalonFXFactory.createDefaultTalon(Ports.ALGAE_ROLLER);

        // Apply configuration (using your constant-provided configurations).
        TalonUtil.applyAndCheckConfiguration(pivotMotor, AlgaeConstants.getPivotConfig());
        TalonUtil.applyAndCheckConfiguration(rollerMotor, AlgaeConstants.getRollerConfig());

        // Set up sensor status signals (refresh at 50 Hz with 0.1 sec latency tolerance).
        Frequency refreshRate = Hertz.of(50);
        positionSignal = pivotMotor.getPosition();
        positionSignal.setUpdateFrequency(refreshRate, 0.1);
        velocitySignal = pivotMotor.getVelocity();
        velocitySignal.setUpdateFrequency(refreshRate, 0.1);
        errorSignal = pivotMotor.getClosedLoopError();
        errorSignal.setUpdateFrequency(refreshRate, 0.1);
        currentSignal = pivotMotor.getStatorCurrent();
        currentSignal.setUpdateFrequency(refreshRate, 0.1);
        voltageSignal = pivotMotor.getMotorVoltage();
        voltageSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointSignal = pivotMotor.getClosedLoopReference();
        setpointSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointVelocitySignal = pivotMotor.getClosedLoopReferenceSlope();
        setpointVelocitySignal.setUpdateFrequency(refreshRate, 0.1);

        // Publish this subsystem to SmartDashboard.
        SmartDashboard.putData("AlgaeArm", this);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();

        // Homing: if the subsystem needs homing, run the state machine.
        if (needsHoming) {
            updateHomingLogic();
        } else {
            homingState = HomingState.IDLE;
        }
        periodicIO.homingState = homingState;

        writePeriodicOutputs();
        logTelemetry();
    }

    /** Read sensor values into our PeriodicIO structure. */
    private void readPeriodicInputs() {
        periodicIO.timestamp = Seconds.of(Timer.getFPGATimestamp());
        periodicIO.position = positionSignal.asSupplier().get();
        periodicIO.velocity = velocitySignal.asSupplier().get();
        periodicIO.current = currentSignal.asSupplier().get();
        periodicIO.voltage = voltageSignal.asSupplier().get();
        periodicIO.closedLoopError = Degrees.of(errorSignal.asSupplier().get());
        periodicIO.goal = targetAngle;
        periodicIO.setpointAngle = Degrees.of(setpointSignal.asSupplier().get());
        periodicIO.setpointRaw = setpointSignal.asSupplier().get();
        periodicIO.setpointVelocityRaw = setpointVelocitySignal.asSupplier().get();
        periodicIO.setpointAngularVelocity = DegreesPerSecond.of(setpointVelocitySignal.asSupplier().get());
    }

    /** Run homing state machine logic. */
    private void updateHomingLogic() {
        switch (homingState) {
            case IDLE:
                // Example condition: if the arm is near its target (and a timeout has passed),
                // start homing.
                boolean timeOut = periodicIO.timestamp.minus(homingStartTime).gt(AlgaeConstants.HOMING_TIMEOUT);
                if (targetAngle.isNear(periodicIO.position, Degrees.of(0.5)) && timeOut) {
                    homingState = HomingState.HOMING;
                    homingStartTime = periodicIO.timestamp;
                }
                break;
            case HOMING:
                // When homing, if the pivot is nearly stalled and the current is high,
                // assume the arm has hit the hard stop.
                boolean velocityStalled = Math.abs(periodicIO.velocity.in(DegreesPerSecond)) < 0.5;
                boolean currentTooHigh = periodicIO.current.in(Amps) >= AlgaeConstants.HOMING_THRESHOLD;
                if (velocityStalled && currentTooHigh) {
                    pivotMotor.setPosition(AlgaeConstants.HOMING_ANGLE); // Zero the sensor at the limit.
                    homingState = HomingState.HOMED;
                }
                break;
            case HOMED:
                // Once homed, clear the flag so normal control resumes.
                needsHoming = false;
                homingState = HomingState.IDLE;
                break;
        }
    }

    /** Write outputs (commands) to the pivot motor. */
    private void writePeriodicOutputs() {
        if (homingState == HomingState.HOMING) {
            // While homing, command a fixed voltage to move the arm.
            pivotMotor.setControl(homingVoltage);
            periodicIO.setpointAngle = Degrees.of(0);
        } else {
            // Normal motion magic control to the target angle.
            // (Since we work in degrees, no conversion is necessary.)
            pivotMotor.setControl(motionMagic.withPosition(targetAngle.in(Degrees)));
        }
    }

    /** Log telemetry values to SmartDashboard. */
    private void logTelemetry() {
        SmartDashboard.putNumber("AlgaeArm/Position_deg", periodicIO.position.in(Degrees));
        SmartDashboard.putNumber("AlgaeArm/Velocity_dps", periodicIO.velocity.in(DegreesPerSecond));
        SmartDashboard.putNumber("AlgaeArm/Current_A", periodicIO.current.in(Amps));
        SmartDashboard.putNumber("AlgaeArm/Voltage_V", periodicIO.voltage.in(Volts));
        SmartDashboard.putNumber("AlgaeArm/ClosedLoopError_deg", periodicIO.closedLoopError.in(Degrees));
        SmartDashboard.putString("AlgaeArm/HomingState", periodicIO.homingState.toString());
        SmartDashboard.putNumber("AlgaeArm/Goal_deg", periodicIO.goal.in(Degrees));
        SmartDashboard.putNumber("AlgaeArm/Setpoint_deg", periodicIO.setpointAngle.in(Degrees));
        // Also log roller motor current for algae detection.
        SmartDashboard.putNumber("AlgaeArm/RollerCurrent_A", rollerMotor.getStatorCurrent().asSupplier().get().in(Amps));
    }

    // --- Public Methods ---

    /** Set the desired target angle for the pivot. */
    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
    }

    /** Get the current pivot angle. */
    public Angle getAngle() {
        return periodicIO.position;
    }

    /** Get the current angular velocity. */
    public AngularVelocity getAngularVelocity() {
        return periodicIO.velocity;
    }

    /** Get the pivot motor current. */
    public Current getPivotCurrent() {
        return periodicIO.current;
    }

    /** Get the roller motor current. */
    public Current getRollerCurrent() {
        return rollerMotor.getStatorCurrent().asSupplier().get();
    }

    /**
     * Returns true if the arm is in a position that would block the CoralManipulator.
     * (Assumes AlgaeConstants.CORAL_THRESHOLD is specified as an Angle.)
     */
    public boolean isBlockingCoralManipulator() {
        return getAngle().gte(AlgaeConstants.CORAL_THRESHOLD);
    }

    /**
     * Returns true if the roller current indicates algae are in contact.
     */
    public boolean hasAlgae() {
        return getRollerCurrent().gte(AlgaeConstants.ALGAE_IN_THRESHOLD);
    }

    /** Set the roller motor speed (using a VoltageOut control). */
    public void setRollerSpeed(Voltage volts) {
        rollerMotor.setControl(new VoltageOut(volts));
    }

    // --- Command methods ---

    /**
     * Returns a command that moves the arm to the specified angle.
     * The command is finished when the current angle is within a tolerance.
     */
    public Command getMoveToAngleCommand(Angle angle) {
        return new Command() {
            @Override
            public void execute() {
                setTargetAngle(angle);
            }

            @Override
            public boolean isFinished() {
                return getAngle().isNear(angle, Degrees.of(0.5));
            }
        };
    }

    /**
     * Returns a command that starts homing the arm.
     * The command is finished once the homing state indicates that homing is complete.
     */
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


    public Command deploy() {
        return getMoveToAngleCommand(AlgaeConstants.ANGLE_UP);
    }

    public Command stow() {
        return new Command() {
            @Override
            public void execute() {
                setTargetAngle(AlgaeConstants.STOW_ANGLE);
            }

            @Override
            public boolean isFinished() {
                return getAngle().isNear(AlgaeConstants.STOW_ANGLE, Degrees.of(0.5));
            }
        };
    }
    public Command spin() {
        return runEnd(() -> setRollerSpeed(AlgaeConstants.INTAKE_OUTPUT), () -> setRollerSpeed(AlgaeConstants.HOLDING_OUTPUT));
    }
}