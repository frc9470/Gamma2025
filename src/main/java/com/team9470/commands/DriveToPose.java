package com.team9470.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

import com.team9470.FieldConstants.Reef;
import com.team9470.TunerConstants;
import com.team9470.subsystems.Swerve;
import com.team9470.util.GeomUtil;
import com.team9470.util.LogUtil;

public class DriveToPose extends Command{
    Swerve drivetrain;
    Supplier<Pose2d> reefPoseSupplier;
    Pose2d reefPose;

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(TunerConstants.maxVelocity, TunerConstants.maxAcceleration);
    TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(TunerConstants.maxAngularVelocity, TunerConstants.maxAngularAcceleration);

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(10, 0, 0, constraints);
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(10, 0, 0, constraints);
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(7, 0, 0, rotationConstraints);

    public DriveToPose(Supplier<Pose2d> reefPoseSuppler, Swerve drivetrain){
        this.reefPoseSupplier = reefPoseSuppler;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        this.reefPose = reefPoseSupplier.get();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d targetPose = getDriveTarget(currentPose, reefPose);
        // Pose2d targetPose = reefPose;
        LogUtil.recordPose2d("Ghost", targetPose);
        
        double xSpeed = pidControllerX.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = pidControllerY.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = pidControllerOmega.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, currentPose.getRotation()
        );
        drivetrain.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getPose().getTranslation().getDistance(reefPose.getTranslation()) <= 0.1 && drivetrain.getPose().getRotation().getDegrees() == reefPose.getRotation().getDegrees();
    }

    private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
        // Final line up
        var offset = robot.relativeTo(goal);
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT =
            MathUtil.clamp(
                (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
                0.0,
                1.0);
        double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength, 0.0, 1.0);
        return goal.transformBy(
            GeomUtil.toTransform2d(
                -shiftXT * 1.5,
                Math.copySign(shiftYT * 1.5 * 0.8, offset.getY())));
  }

}