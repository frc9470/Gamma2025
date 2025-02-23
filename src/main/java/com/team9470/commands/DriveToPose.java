package com.team9470.commands;

import com.team9470.FieldConstants.Reef;
import com.team9470.TunerConstants;
import com.team9470.subsystems.Swerve;
import com.team9470.util.GeomUtil;
import com.team9470.util.LogUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class DriveToPose extends Command {
    private final Swerve drivetrain;
    private final Supplier<Pose2d> reefPoseSupplier;
    private Pose2d reefPose;

    // PID controllers for X, Y and Theta

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(TunerConstants.maxVelocity, TunerConstants.maxAcceleration);
    TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(TunerConstants.maxAngularVelocity, TunerConstants.maxAngularAcceleration);

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(7, 0, 0, constraints);
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(7, 0, 0, constraints);
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(7, 0, 0, rotationConstraints);

    // Feedforward suppliers (default to zero feedforward)
    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    // Default constructor uses no feedforward
    public DriveToPose(Supplier<Pose2d> reefPoseSupplier, Swerve drivetrain) {
        pidControllerOmega.enableContinuousInput(-Math.PI, Math.PI);
        this.reefPoseSupplier = reefPoseSupplier;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Overloaded constructor to inject feedforward from joystick inputs.
    public DriveToPose(Supplier<Pose2d> reefPoseSupplier, Swerve drivetrain,
                       Supplier<Translation2d> linearFF, DoubleSupplier omegaFF) {
        this(reefPoseSupplier, drivetrain);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
    }

    @Override
    public void initialize() {
        this.reefPose = reefPoseSupplier.get();
        Pose2d currentPose = drivetrain.getPose();

        pidControllerX.reset(currentPose.getX());
        pidControllerY.reset(currentPose.getY());
        pidControllerOmega.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        // Determine the target pose using your drive target logic.
        Pose2d targetPose = getDriveTarget(currentPose, reefPose);
        LogUtil.recordPose2d("Ghost", targetPose);
        LogUtil.recordPose2d("Current Reef Pose", reefPose);

        // Calculate feedback speeds from PID controllers.
        double xSpeed = pidControllerX.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = pidControllerY.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = pidControllerOmega.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Combine the X and Y speeds into a translation vector.
        Translation2d driveFeedback = new Translation2d(xSpeed, ySpeed);

        // --- Variable Feedforward Logic ---
        // Get joystick-derived feedforward values.
        Translation2d ffLinear = linearFF.get();
        // Scale feedforward: here we use a factor (3.0) similar to your second DriveToPose.
        double linearScaler = ffLinear.getNorm() * 3.0;
        // Assume DriveConstants.maxLinearSpeed exists (or replace with your max speed, e.g., 3.8).
        Translation2d ffCommand = ffLinear.times(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        // Interpolate between the feedback command and feedforward command.
        Translation2d driveVelocity = driveFeedback.interpolate(ffCommand, linearScaler);

        // For angular velocity, get feedforward from the joystick.
        double ffAngular = omegaFF.getAsDouble();
        double angularScaler = Math.abs(ffAngular) * 3.0;
        // Assume DriveConstants.maxAngularSpeed exists.
        thetaSpeed = MathUtil.interpolate(thetaSpeed, ffAngular * RotationsPerSecond.of(0.75).in(RadiansPerSecond), angularScaler);
        // --- End Feedforward Logic ---

        // Convert the computed speeds to field-relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaSpeed, currentPose.getRotation()
        );
        drivetrain.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        // Finish when both translation and rotation are within tolerances.
        SmartDashboard.putNumber("DriveToPose/Error", drivetrain.getPose().getTranslation().getDistance(reefPose.getTranslation()));
        if (drivetrain.getPose().getTranslation().getDistance(reefPose.getTranslation()) <= 0.02 &&
                Math.abs(drivetrain.getPose().getRotation().getDegrees() - reefPose.getRotation().getDegrees()) <= 2) System.out.println("AUTOALIGN DONE");
        return drivetrain.getPose().getTranslation().getDistance(reefPose.getTranslation()) <= 0.02 &&
                Math.abs(drivetrain.getPose().getRotation().getDegrees() - reefPose.getRotation().getDegrees()) <= 2;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
        // Final lineup logic as originally implemented.
        Translation2d offset = robot.relativeTo(goal).getTranslation();
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT = MathUtil.clamp((yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)), 0.0, 1.0);
        double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength, 0.0, 1.0);
        return goal.transformBy(GeomUtil.toTransform2d(-shiftXT * 1.5, Math.copySign(shiftYT * 1.5 * 0.8, offset.getY())));
    }
}