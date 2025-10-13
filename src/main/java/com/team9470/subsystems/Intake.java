package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Constants.IntakeConstants;
import com.team9470.Ports;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * Intake subsystem for ground intake of coral in the 2025 FRC Reefscape game.
 * Features an arm that moves between retracted and down positions using motion magic,
 * and rollers for intake control. Automatically retracts when coral is detected.
 */
public class Intake extends SubsystemBase {

    // --- Hardware ---
    private final TalonFX armMotor;
    private final TalonFX rollerMotor;
    private final DigitalInput coralSensor = new DigitalInput(Ports.CORAL_BREAK);

    // --- Control objects ---
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final VoltageOut homingVoltage = new VoltageOut(IntakeConstants.HOMING_OUTPUT);

    // --- Status Signals ---
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
    private HomingState homingState = HomingState.HOMING;
    private Time homingStartTime = Seconds.of(0);

    // --- Target angle ---
    private Angle targetAngle = IntakeConstants.HOMING_ANGLE;

    // --- State tracking ---
    private boolean rollersRunning = false;
    private boolean isDown = false;

    // --- Periodic I/O container ---
    public static class PeriodicIO {
        // Inputs
        public Time timestamp;
        public Angle position;
        public AngularVelocity velocity;
        public Current current;
        public Voltage voltage;
        public Angle closedLoopError;

        // Outputs / Telemetry
        public Angle goal;
        public Angle setpointAngle;
        public double setpointRaw;
        public double setpointVelocityRaw;
        public AngularVelocity setpointAngularVelocity;
        public HomingState homingState;
        public boolean rollersRunning;
        public boolean coralDetected;
    }
    private final PeriodicIO periodicIO = new PeriodicIO();

    public Intake() {
        // Create motor instances
        armMotor = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ARM);
        rollerMotor = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLERS);

        // Apply configurations
        TalonUtil.applyAndCheckConfiguration(armMotor, IntakeConstants.getArmConfig());
        TalonUtil.applyAndCheckConfiguration(rollerMotor, IntakeConstants.getRollerConfig());

        // Set up sensor status signals
        Frequency refreshRate = Hertz.of(20);
        positionSignal = armMotor.getPosition();
        positionSignal.setUpdateFrequency(refreshRate, 0.1);
        velocitySignal = armMotor.getVelocity();
        velocitySignal.setUpdateFrequency(refreshRate, 0.1);
        errorSignal = armMotor.getClosedLoopError();
        errorSignal.setUpdateFrequency(refreshRate, 0.1);
        currentSignal = armMotor.getStatorCurrent();
        currentSignal.setUpdateFrequency(refreshRate, 0.1);
        voltageSignal = armMotor.getMotorVoltage();
        voltageSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointSignal = armMotor.getClosedLoopReference();
        setpointSignal.setUpdateFrequency(refreshRate, 0.1);
        setpointVelocitySignal = armMotor.getClosedLoopReferenceSlope();
        setpointVelocitySignal.setUpdateFrequency(refreshRate, 0.1);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();

        // Homing logic
        if (needsHoming) {
            updateHomingLogic();
        } else {
            homingState = HomingState.IDLE;
        }
        periodicIO.homingState = homingState;

        // Auto-retract when coral is detected
        if (isDown && periodicIO.coralDetected) {
            retractAndStopRollers();
        }

        writePeriodicOutputs();

        // Logging data to SmartDashboard / telemetry sinks.
        logTelemetry();
    }

    private void logTelemetry() {
        SmartDashboard.putNumber("Intake/Position", periodicIO.position.in(Degrees));
        SmartDashboard.putString("Intake/HomingState", homingState.name());
        SmartDashboard.putNumber("Intake/Goal", periodicIO.goal.in(Degrees));
        SmartDashboard.putNumber("Intake/Setpoint", periodicIO.setpointAngle.in(Degrees));
    }


    /** Read sensor values into PeriodicIO structure */
    private void readPeriodicInputs() {
        periodicIO.timestamp = Seconds.of(Timer.getFPGATimestamp());
        periodicIO.position = positionSignal.asSupplier().get();
        periodicIO.velocity = velocitySignal.asSupplier().get();
        periodicIO.current = currentSignal.asSupplier().get();
        periodicIO.voltage = voltageSignal.asSupplier().get();
        periodicIO.closedLoopError = Rotations.of(errorSignal.asSupplier().get());
        periodicIO.goal = targetAngle;
        periodicIO.setpointAngle = Rotations.of(setpointSignal.asSupplier().get());
        periodicIO.setpointRaw = setpointSignal.asSupplier().get();
        periodicIO.setpointVelocityRaw = setpointVelocitySignal.asSupplier().get();
        periodicIO.setpointAngularVelocity = RotationsPerSecond.of(setpointVelocitySignal.asSupplier().get());
        periodicIO.rollersRunning = rollersRunning;
        periodicIO.coralDetected = !coralSensor.get(); // Inverted logic - true when beam broken
    }

    /** Run homing state machine logic */
    private void updateHomingLogic() {
        switch (homingState) {
            case IDLE:
                boolean timeOut = periodicIO.timestamp.minus(homingStartTime).gt(IntakeConstants.HOMING_TIMEOUT);
                if (IntakeConstants.HOMING_ANGLE.isNear(periodicIO.position, Degrees.of(5)) && timeOut) {
                    homingState = HomingState.HOMING;
                    homingStartTime = periodicIO.timestamp;
                }
                break;
            case HOMING:
                boolean velocityStalled = Math.abs(periodicIO.velocity.in(DegreesPerSecond)) < 0.5;
                boolean currentTooHigh = periodicIO.current.gte(IntakeConstants.HOMING_THRESHOLD);
                if (velocityStalled && currentTooHigh) {
                    // Zero the sensor at the homing limit
                    armMotor.setPosition(IntakeConstants.HOMING_ANGLE);
                    homingState = HomingState.HOMED;
                }
                break;
            case HOMED:
                needsHoming = false;
                homingState = HomingState.IDLE;
                break;
        }
    }

    /** Write outputs to motors */
    private void writePeriodicOutputs() {
        if (homingState == HomingState.HOMING) {
            armMotor.setControl(homingVoltage);
            periodicIO.setpointAngle = IntakeConstants.HOMING_ANGLE;
        } else {
            armMotor.setControl(
                    motionMagic.withPosition(targetAngle)
                            .withSlot(0)
                            .withEnableFOC(true));
        }
    }

    /** Set the desired target angle for the arm */
    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
        isDown = angle.equals(IntakeConstants.DOWN_ANGLE);
    }

    /** Get the current arm angle */
    public Angle getAngle() {
        return periodicIO.position;
    }

    /** Get the current angular velocity */
    public AngularVelocity getAngularVelocity() {
        return periodicIO.velocity;
    }

    /** Get the arm motor current */
    public Current getArmCurrent() {
        return periodicIO.current;
    }

    /** Start the intake rollers */
    public void startRollers() {
        rollerMotor.setVoltage(IntakeConstants.ROLLER_SPEED.in(Volts));
        rollersRunning = true;
    }

    /** Stop the intake rollers */
    public void stopRollers() {
        rollerMotor.stopMotor();
        rollersRunning = false;
    }

    /** Reverse the intake rollers */
    public void reverseRollers() {
        rollerMotor.setVoltage(IntakeConstants.ROLLER_REVERSE_SPEED.in(Volts));
        rollersRunning = true;
    }

    // TODO: Re-add.
    /** Go to down position and start rollers */
    public void goDownAndStartRollers() {
        setTargetAngle(IntakeConstants.DOWN_ANGLE);
        startRollers();
    }

    /** Retract arm and stop rollers */
    public void retractAndStopRollers() {
        setTargetAngle(IntakeConstants.RETRACTED_ANGLE);
        stopRollers();
    }

    /** Trigger homing sequence */
    public void triggerHoming() {
        needsHoming = true;
        homingState = HomingState.HOMING;
        homingStartTime = periodicIO.timestamp;
    }

    /** Check if coral is detected */
    public boolean hasCoral() {
        return periodicIO.coralDetected;
    }

    /** Check if rollers are running */
    public boolean areRollersRunning() {
        return rollersRunning;
    }

    /** Check if arm is in down position */
    public boolean isDown() {
        return isDown;
    }

    // --- Command Methods ---

    /**
     * Returns a command that moves the arm to the specified angle
     */
    public Command getMoveToAngleCommand(Angle angle) {
        return new Command() {
            @Override
            public void execute() {
                setTargetAngle(angle);
            }
            @Override
            public boolean isFinished() {
                return getAngle().isNear(angle, Degrees.of(2));
            }
        };
    }

    /**
     * Returns a command that starts homing the arm
     */
    public Command getHomingCommand() {
        return new Command() {
            @Override
            public void execute() {
                triggerHoming();
            }
            @Override
            public boolean isFinished() {
                return homingState == HomingState.HOMED;
            }
        };
    }

    /**
     * Returns a command that goes to down position and starts rollers
     */
    public Command goDownAndStartRollersCommand() {
        return this.runOnce(this::goDownAndStartRollers);
    }

    /**
     * Returns a command that retracts arm and stops rollers
     */
    public Command retractAndStopRollersCommand() {
        return this.runOnce(this::retractAndStopRollers);
    }

    /**
     * Returns a command that goes down and starts rollers, then automatically retracts when coral is detected
     */
    public Command autoIntakeCommand() {
        return goDownAndStartRollersCommand()
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        // Keep running until coral is detected
                    }
                    @Override
                    public boolean isFinished() {
                        return hasCoral();
                    }
                })
                .andThen(retractAndStopRollersCommand());
    }

    /**
     * Returns a command that runs rollers continuously
     */
    public Command runRollersCommand() {
        return this.run(this::startRollers);
    }

    /**
     * Returns a command that stops rollers
     */
    public Command stopRollersCommand() {
        return this.runOnce(this::stopRollers);
    }

    /**
     * Returns a command that reverses rollers
     */
    public Command reverseRollersCommand() {
        return this.run(this::reverseRollers);
    }

    /**
     * Returns a command that goes to retracted position
     */
    public Command retractCommand() {
        return getMoveToAngleCommand(IntakeConstants.RETRACTED_ANGLE);
    }

    /**
     * Returns a command that goes to down position
     */
    public Command goDownCommand() {
        return getMoveToAngleCommand(IntakeConstants.DOWN_ANGLE);
    }
}
