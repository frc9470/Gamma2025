package com.team9470.commands;

import com.team9470.Constants;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Meters;

public class AutoScoring {


    public Command autoScore(CoralObjective coralObjective, Superstructure superstructure){

        return new DriveToPose(coralObjective.getScoringPose())
                .alongWith(new WaitUntilCommand(() -> closeEnough(Constants.DriverAssistConstants.RAISE_DISTANCE)))
                .andThen(superstructure.raiseAndStow(coralObjective.level))
                .andThen(superstructure.score());

    }

    private boolean closeEnough(CoralObjective objective, Distance distance){
        Pose2d scoringPose = objective.getScoringPose();
        Pose2d currentPose = Swerve.getInstance().getReefPose(objective.getFace(), scoringPose);
        return currentPose.getTranslation().getDistance(scoringPose.getTranslation()) < distance.in(Meters);
    }

    public record CoralObjective(int branchId, int level){
        public Pose2d getScoringPose(){
            return Constants.DriverAssistConstants.getReefPositions(DriverStation.Alliance.Red)[branchId];
        }

        public int getFace(){
            // branch id is 1 indexed, faces are 0 indexed and there are two branches per face
            return (branchId - 1) / 2;
        }

        public CoralObjective updateBranchId(int branchId){
            return new CoralObjective(branchId, level);
        }

        public CoralObjective updateLevel(int level){
            return new CoralObjective(branchId, level);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setControl(
                robotCentricRequest.withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }


}
