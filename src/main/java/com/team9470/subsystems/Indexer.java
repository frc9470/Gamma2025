package com.team9470.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.DelayedBoolean;
import com.team9470.Constants.IndexerConstants;
import com.team9470.Ports;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer subsystem for handling coral in the 2025 FRC Reefscape game.
 * Uses two motors running in opposite directions to move coral through the system.
 * Detects coral presence using current monitoring and beam break sensors.
 */
public class Indexer extends SubsystemBase {

    private final TalonFX indexerMotor1 = TalonFXFactory.createDefaultTalon(Ports.INDEXER_1);
    private final TalonFX indexerMotor2 = TalonFXFactory.createDefaultTalon(Ports.INDEXER_2);
    

    private final DigitalInput coralSensor = new DigitalInput(Ports.INDEXER_SENSOR); //beam break
    
    /** STATUS SIGNALS for current monitoring */
    private final StatusSignal<Current> motor1CurrentSignal = indexerMotor1.getStatorCurrent();
    private final StatusSignal<Current> motor2CurrentSignal = indexerMotor2.getStatorCurrent();
    
    /** Delayed boolean for coral detection debouncing */
    private final DelayedBoolean coralDetected = new DelayedBoolean(
        Timer.getFPGATimestamp(), 
        IndexerConstants.CORAL_DETECTION_TIMEOUT, 
        false
    );
    
    /** State tracking */
    private boolean isRunning = false;
    private boolean coralPresent = false;

    public Indexer() {
        // Apply motor configurations
        TalonUtil.applyAndCheckConfiguration(indexerMotor1, IndexerConstants.getMotorConfig());
        TalonUtil.applyAndCheckConfiguration(indexerMotor2, IndexerConstants.getMotorConfig());
        
        // Set up status signals for current monitoring
        motor1CurrentSignal.setUpdateFrequency(50, 0.1);
        motor2CurrentSignal.setUpdateFrequency(50, 0.1);
        
        // Set default command to stop the indexer
        setDefaultCommand(stopCommand());
    }

    @Override
    public void periodic() {
        // Update coral detection based on current and beam break
        updateCoralDetection();
        
        // Update delayed boolean for debouncing
        coralDetected.update(Timer.getFPGATimestamp(), coralPresent);

        // Push values to SmartDashboard
        logTelemetry();
    }

    private void logTelemetry() {
        SmartDashboard.putBoolean("Indexer/hasCoral", hasCoral());
    }

    /**
     * Updates coral detection based on beam break sensor as primary method,
     * with current monitoring as fallback
     */
    private void updateCoralDetection() {
        // Primary detection: Beam break sensor (inverted logic - true when beam is broken)
        boolean beamBreak = !coralSensor.get();
        
        // Fallback detection: Current monitoring
        Current motor1Current = motor1CurrentSignal.asSupplier().get();
        Current motor2Current = motor2CurrentSignal.asSupplier().get();
        boolean highCurrent1 = motor1Current.gte(IndexerConstants.CORAL_DETECTION_CURRENT);
        boolean highCurrent2 = motor2Current.gte(IndexerConstants.CORAL_DETECTION_CURRENT);
        boolean highCurrentFallback = highCurrent1 || highCurrent2;
        
        // Coral is present if beam break is triggered (primary) OR current fallback is active
        coralPresent = beamBreak || highCurrentFallback;
    }

    /**
     * Starts the indexer with both motors running in opposite directions
     */
    public void startIndexer() {
        indexerMotor1.setVoltage(IndexerConstants.INDEXER_SPEED.in(Volts));
        indexerMotor2.setVoltage(-IndexerConstants.INDEXER_SPEED.in(Volts));
        isRunning = true;
    }

    /**
     * Stops the indexer by stopping both motors
     */
    public void stopIndexer() {
        indexerMotor1.stopMotor();
        indexerMotor2.stopMotor();
        isRunning = false;
    }

    /**
     * Runs the indexer in reverse (both motors opposite to normal direction)
     */
    public void reverseIndexer() {
        indexerMotor1.setVoltage(IndexerConstants.INDEXER_REVERSE_SPEED.in(Volts));
        indexerMotor2.setVoltage(-IndexerConstants.INDEXER_REVERSE_SPEED.in(Volts));
        isRunning = true;
    }

    /**
     * Holds the indexer with a small voltage to prevent coral from falling out
     */
    public void holdIndexer() {
        indexerMotor1.setVoltage(IndexerConstants.INDEXER_HOLD_SPEED.in(Volts));
        indexerMotor2.setVoltage(-IndexerConstants.INDEXER_HOLD_SPEED.in(Volts));
        isRunning = false;
    }

    /**
     * Determines whether a coral is currently in the indexer
     * Uses debounced detection to prevent false positives
     * 
     * @return true if coral is detected in the indexer
     */
    public boolean hasCoral() {
        return coralDetected.update(Timer.getFPGATimestamp(), coralPresent);
    }

    /**
     * Returns a command that handles the complete intake-to-output cycle:
     * 1. Wits for coral to be detected (intake)
     * 2. Continues running until coral leaves the indexer (output to robot)
     * This is the main command
     */
    public Command intakeToOutputCommand() {
        return this.run(this::startIndexer)
                .until(this::hasCoral)
                .andThen(this.run(this::startIndexer))
                .until(() -> !hasCoral())
                .andThen(this::stopIndexer);
    }

    public Command intakeCommand() {
        return this.run(this::startIndexer);
    }

    /**
     * Only intakes, and then holds coral
     * @return command
     */
    public Command intakeAndHoldCommand(){
        return this.run(this::startIndexer)
        .until(this::hasCoral)
        .andThen(this.run(this::holdIndexer));
    }

    /**
     * Outputs coral into robot, assuming coral is being held right now
     * @return command
     */
    public Command outputCommand(){
        return this.run(this::startIndexer)
        .until(() -> !hasCoral())
        .andThen(this::stopIndexer);
    }

    /**
     * Returns a command that runs the indexer continuously
     */
    public Command runCommand() {
        return this.run(this::startIndexer);
    }

    /**
     * Returns a command that runs the indexer in reverse
     */
    public Command reverseCommand() {
        return this.run(this::reverseIndexer);
    }

    /**
     * Returns a command that holds the indexer (small voltage to prevent coral from falling)
     */
    public Command holdCommand() {
        return this.run(this::holdIndexer);
    }

    /**
     * Returns a command that holds the indexer (small voltage to prevent coral from falling)
     */
    public Command stopCommand() {
        return this.run(this::stopIndexer);
    }
}
