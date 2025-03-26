package com.team9470.subsystems.sim;

import com.google.gson.Gson;
import com.team9470.Telemetry;
import com.team9470.commands.AutoScoring;
import com.team9470.util.Util;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBoardSub extends SubsystemBase {
    private static ButtonBoardSub mInstance;
    private final Telemetry logger;
    private final AutoScoring autoScoringCommand;
    private final IntegerSubscriber branchSub;
    private final IntegerSubscriber levelSub;
    public int curBranch;
    public int curLevel;
    public Gson gson = new Gson();

    public static ButtonBoardSub getInstance(Telemetry logger, AutoScoring autoScoringCommand) {
        if (mInstance == null) {
            mInstance = new ButtonBoardSub(logger, autoScoringCommand);
        }

        return mInstance;
    }

    public ButtonBoardSub(Telemetry logger, AutoScoring autoScoringCommand) {
        this.logger = logger;
        this.autoScoringCommand = autoScoringCommand;

        branchSub = logger.branch.getTopic().subscribe(0);
        levelSub = logger.level.getTopic().subscribe(0);
    }

    @Override
    public void periodic() {
        int level = (int) levelSub.get();
        int branch = (int) branchSub.get();
        if(branch != curBranch){
            curBranch = branch;
            if(autoScoringCommand.getCorals()[curLevel-1][curBranch]){
                autoScoringCommand.addCoral(branch, curLevel-1);
            }
            else{
                autoScoringCommand.removeCoral(branch, curLevel-1);
            }
        }
        // System.out.println(branch);
        // System.out.println(level);
    }
}