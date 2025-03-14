package com.team9470.commands;

import com.team9470.Constants;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Set;

import static edu.wpi.first.units.Units.Meters;

public class AutoScoring {

    private ScoringState scoringState = ScoringState.DEFAULT;
    private static Swerve drivetrain;

    public AutoScoring(Swerve drivetrain){
        this.drivetrain = drivetrain;
    }

    public Command autoScore(Superstructure superstructure) {
        CoralObjective coralObjective = scoringState.getOptimalObjective();
        return new DeferredCommand(() -> {
            Command driveToScore = new DriveToPose(coralObjective::getScoringPose, drivetrain)
                    .alongWith(
                            new WaitUntilCommand(() -> closeEnough(coralObjective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                    .andThen(superstructure.waitForIntake().asProxy())
                                    .andThen(superstructure.raise(scoringState.getLevel()).asProxy())
                    );
            System.out.println("Scoring with coralobjective: " + coralObjective);
            return driveToScore.andThen(superstructure.getCoral().scoreCommand().asProxy());
        }, Set.of(drivetrain));
    }

    public static Command autoScore(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                .andThen(superstructure.waitForIntake().asProxy())
                                .andThen(new DeferredCommand(() -> superstructure.raise(objective.level), Set.of(superstructure)).asProxy())
                );
        if(objective.level == 1){
            return driveToScore.andThen(superstructure.getCoral().scoreCommand().asProxy());
        } else return driveToScore.andThen(superstructure.getCoral().scoreCommand().asProxy());
    }

    public static Command autoScoreWithTimeout(Superstructure superstructure, CoralObjective objective, Swerve drivetrain, double timeout) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                .andThen(new DeferredCommand(() -> superstructure.raise(objective::level), Set.of(superstructure)).asProxy())
                );
        return driveToScore.andThen(superstructure.getCoral().scoreCommand().withTimeout(timeout).asProxy());
    }
    public Command autoScoreNoDrive(Superstructure superstructure) {
        return new DeferredCommand(() -> superstructure.raise(scoringState.getLevel()), Set.of(superstructure)).andThen(superstructure.score());
    }

    private Pose2d computeDriveBackPose(Pose2d scoringPose) {
        // Drive backward 1 meter relative to the scoring pose.
        Translation2d backOffset = new Translation2d(-.5, 0.0).rotateBy(scoringPose.getRotation());
        return new Pose2d(scoringPose.getTranslation().plus(backOffset), scoringPose.getRotation());
    }

    public Command driveBack(){
        Pose2d scoringPose = scoringState.getOptimalObjective().getScoringPose();
        Pose2d driveBackPose = computeDriveBackPose(scoringPose);

        return new DriveToPose(() -> driveBackPose, drivetrain);

    }

    public Command autoAlgae(Superstructure superstructure){
        CoralObjective coralObjective = scoringState.getOptimalObjective();
        return new DriveToPose(() -> coralObjective.getScoringPose(), drivetrain)
                .alongWith(new WaitUntilCommand(() -> closeEnough(coralObjective, Constants.DriverAssistConstants.RAISE_DISTANCE)))
                .andThen(new DeferredCommand(() -> superstructure.dealgify(coralObjective.getAlgaeLevel()), Set.of(superstructure)));
    }

    public Command autoAlgaeNoDrive(Superstructure superstructure){
        return new DeferredCommand(() -> superstructure.dealgify(scoringState.getOptimalObjective().getAlgaeLevel()), Set.of(superstructure));
    }

    private static boolean closeEnough(CoralObjective objective, Distance distance){
        Pose2d scoringPose = objective.getScoringPose();
        Pose2d currentPose = Swerve.getInstance().getPose();
        return currentPose.getTranslation().getDistance(scoringPose.getTranslation()) < distance.in(Meters);
    }


    // allows op to choose level
    public void overrideLevel(int level) {
        scoringState = scoringState.updateLevel(level);
    }

    // removes operator override of level
    public void removeOverride(){
        scoringState = scoringState.updateLevel(0);
    }

    public void addCoral(int branchId, int level){
        scoringState = scoringState.updateCoral(branchId, level, true);
    }

    public void removeCoral(int branchId, int level){
        scoringState = scoringState.updateCoral(branchId, level, false);
    }

    // stores scored coral and current level
    // if curLevel = 0, it will automatically set it
    public record ScoringState(boolean[][] corals, int curLevel){
        public static final ScoringState DEFAULT = new ScoringState(new boolean[4][12], 0);

        public ScoringState updateCoral(int branchId, int level, boolean newValue){
            corals[level-1][branchId] = newValue;
            return new ScoringState(corals, curLevel);
        }

        public int getLevel(){
            if(curLevel == 0){
                for(int i = 3; i >= 0 ; i--){
                    for(int j = 0; j < 12; j++){
                        if(!corals[i][j]){
                            return i+1;
                        }
                    }
                }
                return 1;
            }
            return curLevel;
        }

        public ScoringState updateLevel(int newLevel){
            return new ScoringState(corals, newLevel);
        }

        public CoralObjective getOptimalObjective(){
            Pose2d[] reefPoses = Constants.DriverAssistConstants.getReefPositions();
            Pose2d currentPose = drivetrain.getPose();
            int level = getLevel();

            // Find the closest reef pose.
            double shortestDistance = Double.MAX_VALUE;
            int closestPoseId = -1;
            for (int i = 0; i < 12; i++) {
                if(corals[level-1][i]){
                    continue;
                }
                Pose2d pose = reefPoses[i];
                double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
                if (distance < shortestDistance) {
                    shortestDistance = distance;
                    closestPoseId = i;
                }
            }
            return new CoralObjective(closestPoseId, level);
        }
    }
    
    // Starting facing the driver station, moving counterclockwise
    public record CoralObjective(int branchId, int level){
        public static final CoralObjective NONE = new CoralObjective(0, 0);

        public Pose2d getScoringPose(){
            if (level == 1) return Constants.DriverAssistConstants.getL1Pose(this);
            return Constants.DriverAssistConstants.getReefPositions()[branchId];
        }

        public int getFace(){
            return (branchId) / 2;
        }

        public int getAlgaeLevel(){
            // starting with (0, 1) = 3, then (2, 3) = 2, each reef alternates level between 2 and 3
            return (branchId)/2 % 2 == 0 ? 3 : 2;
        }
    }
}
