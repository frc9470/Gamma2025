package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Constants.ArmConstants;
import com.team9470.Ports;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * Arm subsystem for handling both coral and algae intake and scoring in the 2025 FRC Reefscape game.
 * Features motion magic control for precise positioning and roller control for item manipulation.
 * Automatically detects items via current monitoring and maintains hold pressure.
 */
public class Arm extends SubsystemBase {
    // --- Hardware ---
    private final TalonFX pivotMotor = TalonFXFactory.createDefaultTalon(Ports.ARM_ANGLE);
    private final TalonFX rollerMotor = TalonFXFactory.createDefaultTalon(Ports.ARM_ROLLERS);

    // --- Control objects ---
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final VoltageOut homingVoltage = new VoltageOut(ArmConstants.HOMING_OUTPUT);

    // --- Status Signals ---
    private final StatusSignal<Angle> positionSignal = pivotMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocitySignal = pivotMotor.getVelocity();
    private final StatusSignal<Current> pivotCurrentSignal = pivotMotor.getStatorCurrent();
    private final StatusSignal<Current> rollerCurrentSignal = rollerMotor.getStatorCurrent();

    // --- State tracking ---
    private boolean rollersRunning = false;
    private boolean hasItem = false;
    private boolean holdingItem = false;
    
    // Item type tracking
    private boolean holdingCoral = false;
    private boolean holdingAlgae = false;

    // --- Homing logic ---
    private enum HomingState {
        IDLE,
        HOMING,
        HOMED
    }
    private boolean needsHoming = true;
    private HomingState homingState = HomingState.HOMING;
    private Time homingStartTime = Seconds.of(0);

    // --- Target angle ---
    private Angle targetAngle = ArmConstants.HOMING_ANGLE;

    // --- Periodic I/O container ---
    public static class PeriodicIO {
        // Inputs
        public Time timestamp;
        public Angle position;
        public AngularVelocity velocity;
        public Current pivotCurrent;
        public Current rollerCurrent;

        // Outputs / Telemetry
        public Angle goal;
        public HomingState homingState;
        public boolean rollersRunning;
        public boolean hasItem;
        public boolean holdingItem;
    }
    private final PeriodicIO periodicIO = new PeriodicIO();


    public Arm() {
        // Apply motor configurations
        TalonUtil.applyAndCheckConfiguration(pivotMotor, ArmConstants.getPivotConfig());
        TalonUtil.applyAndCheckConfiguration(rollerMotor, ArmConstants.getRollerConfig());

        // Set up sensor status signals
        Frequency refreshRate = Hertz.of(20);
        positionSignal.setUpdateFrequency(refreshRate, 0.1);
        velocitySignal.setUpdateFrequency(refreshRate, 0.1);
        pivotCurrentSignal.setUpdateFrequency(refreshRate, 0.1);
        rollerCurrentSignal.setUpdateFrequency(refreshRate, 0.1);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();

        // Homing logic
        if (needsHoming) {
            updateHomingLogic();
        } else if (homingState != HomingState.HOMED) {
            homingState = HomingState.IDLE;
        }
        periodicIO.homingState = homingState;

        // Item detection and hold logic
        updateItemDetection();

        writePeriodicOutputs();
    }

    /** Read sensor values into PeriodicIO structure */
    private void readPeriodicInputs() {
        periodicIO.timestamp = Seconds.of(Timer.getFPGATimestamp());
        periodicIO.position = positionSignal.asSupplier().get();
        periodicIO.velocity = velocitySignal.asSupplier().get();
        periodicIO.pivotCurrent = pivotCurrentSignal.asSupplier().get();
        periodicIO.rollerCurrent = rollerCurrentSignal.asSupplier().get();
        periodicIO.goal = targetAngle;
        periodicIO.rollersRunning = rollersRunning;
        periodicIO.hasItem = hasItem;
        periodicIO.holdingItem = holdingItem;
    }

    /** Run homing state machine logic */
    private void updateHomingLogic() {
        switch (homingState) {
            case IDLE:
                boolean timeOut = periodicIO.timestamp.minus(homingStartTime).gt(ArmConstants.HOMING_TIMEOUT);
                if (ArmConstants.HOMING_ANGLE.isNear(periodicIO.position, Degrees.of(5)) && timeOut) {
                    homingState = HomingState.HOMING;
                    homingStartTime = periodicIO.timestamp;
                }
                break;
            case HOMING:
                boolean velocityStalled = Math.abs(periodicIO.velocity.in(DegreesPerSecond)) < 0.5;
                boolean currentTooHigh = periodicIO.pivotCurrent.gte(ArmConstants.HOMING_THRESHOLD);
                if (velocityStalled && currentTooHigh) {
                    // Zero the sensor at the homing limit
                    pivotMotor.setPosition(ArmConstants.HOMING_ANGLE);
                    homingState = HomingState.HOMED;
                    needsHoming = false;
                }
                break;
            case HOMED:
                // Stay in HOMED state until explicitly re-homed
                break;
        }
    }

    /** Update item detection and hold logic */
    private void updateItemDetection() {
        // Check if rollers are drawing high current (indicating item resistance)
        boolean rollerHighCurrent = periodicIO.rollerCurrent.gte(ArmConstants.ITEM_DETECTION_CURRENT);
        
        // Item is detected based on roller current draw
        hasItem = rollerHighCurrent;
        
        // If we have an item and rollers are running, switch to hold mode
        if (hasItem && rollersRunning && !holdingItem) {
            holdingItem = true;
            holdItem();
        }
    }

    /** Write outputs to motors */
    private void writePeriodicOutputs() {
        if (homingState == HomingState.HOMING) {
            pivotMotor.setControl(homingVoltage);
        } else {
            pivotMotor.setControl(
                    motionMagic.withPosition(targetAngle)
                            .withSlot(0)
                            .withEnableFOC(true));
        }
    }

    /** Set the desired target angle for the arm */
    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
        double minDegrees = ArmConstants.MIN_ANGLE.in(Degrees);
        double maxDegrees = ArmConstants.MAX_ANGLE.in(Degrees);
        double clamped = MathUtil.clamp(angle.in(Degrees), minDegrees, maxDegrees);
        targetAngle = Degrees.of(clamped);
    }

    /** Get the current arm angle */
    public Angle getAngle() {
        return periodicIO.position;
    }

    /** Get the current angular velocity */
    public AngularVelocity getAngularVelocity() {
        return periodicIO.velocity;
    }

    /** Get the pivot motor current */
    public Current getPivotCurrent() {
        return periodicIO.pivotCurrent;
    }

    /** Get the roller motor current */
    public Current getRollerCurrent() {
        return periodicIO.rollerCurrent;
    }

    /** Start intake rollers */
    public void startIntake() {
        rollerMotor.setVoltage(ArmConstants.ROLLER_INTAKE_SPEED.in(Volts));
        rollersRunning = true;
        holdingItem = false;
    }

    /** Start output rollers */
    public void startOutput() {
        rollerMotor.setVoltage(ArmConstants.ROLLER_OUTPUT_SPEED.in(Volts));
        rollersRunning = true;
        holdingItem = false;
    }

    /** Hold item with stalling voltage */
    public void holdItem() {
        rollerMotor.setVoltage(ArmConstants.ROLLER_HOLD_SPEED.in(Volts));
        rollersRunning = true;
        holdingItem = true;
    }

    /** Stop rollers */
    public void stopRollers() {
        rollerMotor.stopMotor();
        rollersRunning = false;
        holdingItem = false;
    }

    /** Trigger homing sequence */
    public void triggerHoming() {
        needsHoming = true;
        homingState = HomingState.HOMING;
        homingStartTime = Seconds.of(Timer.getFPGATimestamp());;
    }

    /** Check if item is detected */
    public boolean hasItem() {
        return hasItem;
    }

    /** Check if rollers are running */
    public boolean areRollersRunning() {
        return rollersRunning;
    }

    /** Check if holding item */
    public boolean isHoldingItem() {
        return holdingItem;
    }

    /** Set item type being held */
    public void setHoldingCoral(boolean holding) {
        holdingCoral = holding;
        holdingAlgae = !holding;
    }

    /** Set item type being held */
    public void setHoldingAlgae(boolean holding) {
        holdingAlgae = holding;
        holdingCoral = !holding;
    }

    /** Check if holding coral */
    public boolean isHoldingCoral() {
        return holdingCoral;
    }

    /** Check if holding algae */
    public boolean isHoldingAlgae() {
        return holdingAlgae;
    }

    // --- Command Methods ---

    /**
     * Returns a command that moves the arm to the specified angle
     */
    public Command getMoveToAngleCommand(Angle angle) {
        return this.run(() -> setTargetAngle(angle))
                .until(() -> getAngle().isNear(angle, Degrees.of(2)));
    }

    /**
     * Returns a command that starts homing the arm
     */
    public Command getHomingCommand() {
        return this.runOnce(this::triggerHoming)
            .andThen(Commands.waitUntil(() -> homingState == HomingState.HOMED));
    }

    // --- Intake Commands ---

    public Command coralIntakeCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_INTAKE_ANGLE)
                .andThen(this.run(this::startIntake));
    }

    public Command algaeGroundIntakeCommand() {
        return getMoveToAngleCommand(ArmConstants.ALGAE_GROUND_INTAKE_ANGLE)
                .andThen(this.run(this::startIntake));
    }

    public Command algaeReefIntakeCommand() {
        return getMoveToAngleCommand(ArmConstants.ALGAE_REEF_INTAKE_ANGLE)
                .andThen(this.run(this::startIntake));
    }

    // --- Coral Scoring Commands ---

    public Command coralL4BeforeScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_L4_BEFORE_SCORING);
    }

    public Command coralL4ScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_L4_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !hasItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    public Command coralL3BeforeScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_L3_BEFORE_SCORING);
    }

    public Command coralL3ScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_L3_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !hasItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    public Command coralL2BeforeScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_L2_BEFORE_SCORING);
    }

    public Command coralL2ScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_L2_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !hasItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    public Command coralL1ScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.CORAL_L1_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !hasItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    // --- Algae Scoring Commands ---

    public Command algaeBargeBeforeScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.ALGAE_BARGE_BEFORE_SCORING);
    }

    public Command algaeBargeScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.ALGAE_BARGE_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !hasItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    public Command algaeProcessorScoringCommand() {
        return getMoveToAngleCommand(ArmConstants.ALGAE_PROCESSOR_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !hasItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    // --- General Commands ---

    public Command runRollersCommand() {
        return this.run(this::startIntake);
    }

    public Command stopRollersCommand() {
        return this.runOnce(this::stopRollers);
    }

    public Command holdItemCommand() {
        return this.run(this::holdItem);
    }
}
