package com.team9470.commands;

public class AutoScoring {


//    public Command getAutoScoringCommand(int reef, int level, Swerve swerve, Superstructure superstructure){
//        return swerve.getPathfindingCommand().
//    }

    public int getAlgaeLevel(int reef){
        // starting with (1, 2) = 3, then (3, 4) = 2, each reef alternates level between 2 and 3
        return (reef+1)/2 % 2 == 0 ? 2 : 3;
    }
}
