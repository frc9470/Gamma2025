package com.team9470.commands;

import com.team9470.Constants;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Meters;

public class AutoScoring {

    private CoralObjective coralObjective = AutoScoring.CoralObjective.NONE;

    public Command autoScore(Superstructure superstructure){

        return new DriveToPose(coralObjective.getScoringPose())
                .alongWith(new WaitUntilCommand(() -> closeEnough(coralObjective, Constants.DriverAssistConstants.RAISE_DISTANCE)))
                .andThen(superstructure.raiseAndStow(coralObjective.level))
                .andThen(superstructure.score());

    }

    public Command autoAlgae(Superstructure superstructure){
        return new DriveToPose(coralObjective.getScoringPose())
                .alongWith(new WaitUntilCommand(() -> closeEnough(coralObjective, Constants.DriverAssistConstants.RAISE_DISTANCE)))
                .andThen(superstructure.dealgify(coralObjective.getAlgaeLevel()));
    }

    private boolean closeEnough(CoralObjective objective, Distance distance){
        Pose2d scoringPose = objective.getScoringPose();
        Pose2d currentPose = Swerve.getInstance().getReefPose(objective.getFace(), scoringPose);
        return currentPose.getTranslation().getDistance(scoringPose.getTranslation()) < distance.in(Meters);
    }

    // Update the reef value used for automatic level selection.
    public void setBranch(int branchId) {
        coralObjective = coralObjective.updateBranchId(branchId);
    }

    public void setLevel(int level) {
        coralObjective = coralObjective.updateLevel(level);
    }

    public record CoralObjective(int branchId, int level){
        public static final CoralObjective NONE = new CoralObjective(0, 0);

        public Pose2d getScoringPose(){
            return Constants.DriverAssistConstants.getReefPositions(DriverStation.Alliance.Red)[branchId];
        }

        public int getFace(){
            return (branchId) / 2;
        }

        public int getAlgaeLevel(){
            // starting with (1, 2) = 3, then (3, 4) = 2, each reef alternates level between 2 and 3
            return (branchId+2)/2 % 2 == 0 ? 2 : 3;
        }


        public CoralObjective updateBranchId(int branchId){
            return new CoralObjective(branchId, level);
        }

        public CoralObjective updateLevel(int level){
            return new CoralObjective(branchId, level);
        }
    }
}
