package com.team9470.subsystems.sim;

import com.google.gson.Gson;
import com.team9470.Telemetry;
import com.team9470.util.Util;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBoardSub extends SubsystemBase {
    private static ButtonBoardSub mInstance;
    private final Telemetry logger;
    private final IntegerSubscriber branchSub;
    private final IntegerSubscriber levelSub;
    public int branch;
    public int level;
    public Gson gson = new Gson();

    public static ButtonBoardSub getInstance(Telemetry logger) {
        if (mInstance == null) {
            mInstance = new ButtonBoardSub(logger);
        }

        return mInstance;
    }

    public ButtonBoardSub(Telemetry logger) {
        this.logger = logger;

        branchSub = logger.branch.getTopic().subscribe(0);
        levelSub = logger.level.getTopic().subscribe(0);
    }

    @Override
    public void periodic() {
        branch = (int) branchSub.get();
        level = (int) levelSub.get();
        // System.out.println(branch);
        // System.out.println(level);
    }
}