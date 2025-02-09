package com.team9470.subsystems;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.team9470.Constants.DriverAssistConstants;
import com.team9470.TunerConstants;
import com.team9470.TunerConstants.TunerSwerveDrivetrain;
import com.team9470.subsystems.vision.VisionPoseAcceptor;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;



/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {

    private final VisionPoseAcceptor visionPoseAcceptor = new VisionPoseAcceptor();
    private static Swerve instance;

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    /* Swerve requests */
    private final SwerveRequest.ApplyFieldSpeeds    applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);


    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> setControl(translationCharacterization.withVolts(output)),
                    null,
                    this
            )
    );
    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(steerCharacterization.withVolts(volts)),
                    null,
                    this
            )
    );
    /* The SysId routine to test */
    private final SysIdRoutine sysIdRoutineToApply = sysIdRoutineSteer;
    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this
            )
    );
    
    private static RobotConfig config;

    private double m_lastSimTime;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        instance = this;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        configAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        instance = this;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        configAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        instance = this;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        configAutoBuilder();
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(Choreo.TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                () -> getState().Pose,
                this::resetPose,
                this::followPath,
                true,
                this,
                trajLogger
        );
    }

    // For pathplanner
    public void configAutoBuilder(){
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(7.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void setChassisSpeeds(ChassisSpeeds speeds){
        setControl(
                robotCentricRequest.withVelocityX(speeds.vxMetersPerSecond)
                                   .withVelocityY(speeds.vyMetersPerSecond)
                                   .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    public Command getPathfindingCommand(Pose2d targetPose){
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                TunerConstants.maxVelocity, TunerConstants.maxAcceleration,
                Units.degreesToRadians(TunerConstants.maxAngularVelocity), Units.degreesToRadians(TunerConstants.maxAngularAcceleration));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );

        return pathfindingCommand;
    }

    public Command pathfindClosestReefPos() {
        return new Command() {
            private Command pathfindingCommand;

            @Override
            public void initialize() {
                // Select the appropriate reef positions.
                // (You can add alliance-specific logic if needed.)
                Pose2d[] reefPoses = DriverAssistConstants.BLUE_REEF_POSITIONS;

                // Get the current robot pose at initialization.
                Pose2d currentPose = getPose();

                // Find the closest reef pose.
                double shortestDistance = Double.MAX_VALUE;
                Pose2d closestPose = null;
                for (Pose2d pose : reefPoses) {
                    double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
                    if (distance < shortestDistance) {
                        shortestDistance = distance;
                        closestPose = pose;
                    }
                }

                // Create the pathfinding command using the closest pose.
                pathfindingCommand = getPathfindingCommand(closestPose);
                // Initialize the pathfinding command.
                pathfindingCommand.initialize();
            }

            @Override
            public void execute() {
                // Delegate execution to the pathfinding command.
                pathfindingCommand.execute();
            }

            @Override
            public boolean isFinished() {
                return pathfindingCommand.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                pathfindingCommand.end(interrupted);
            }
        };
    }

    /**
     * 
     * @param posID ID os reef pos from 0-5
     * @return pathfinding command
     */
    public Command pathfindReefPos(int posId){
        Pose2d[] reefPoses;
        // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        //     reefPoses = DriverAssistConstants.RED_REEF_POSITIONS;
        // }
        // else{
        //     reefPoses = DriverAssistConstants.BLUE_REEF_POSITIONS;
        // }
        reefPoses = DriverAssistConstants.BLUE_REEF_POSITIONS;

        return getPathfindingCommand(reefPoses[posId]);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
                pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
                pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading
        );

        setControl(
                applyFieldSpeeds.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? RED_ALLIANCE_PERSPECTIVE_ROTATION
                                : BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    public Pose2d getPose()
    {
        return getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds(){
        // Get the current swerve module states
        SwerveModuleState[] moduleStates = getState().ModuleStates;

        // Convert module states to chassis speeds using the drivetrain kinematics
        SwerveDriveKinematics kinematics = getKinematics();
        return kinematics.toChassisSpeeds(moduleStates);
    }

    public static Swerve getInstance(){
        if(instance == null){
            instance = TunerConstants.createDrivetrain();
        }
        return instance;
    }

}
