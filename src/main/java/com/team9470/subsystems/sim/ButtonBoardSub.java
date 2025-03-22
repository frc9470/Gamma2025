package com.team9470.subsystems.sim;

import com.google.gson.Gson;
import com.team9470.Telemetry;
import com.team9470.util.Util;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBoardSub extends SubsystemBase {
    private static ButtonBoardSub mInstance;
    private final Telemetry logger;
    private final StringSubscriber coralInfoSub;
    public boolean[][] coralInfo;
    public Gson gson = new Gson();

    public static ButtonBoardSub getInstance(Telemetry logger) {
        if (mInstance == null) {
            mInstance = new ButtonBoardSub(logger);
        }

        return mInstance;
    }

    public ButtonBoardSub(Telemetry logger) {
        this.logger = logger;

        coralInfoSub = logger.coralInfo.getTopic().subscribe("");
    }

    @Override
    public void periodic() {
//        coralInfo = Util.BooleanJSONArrayStringTo2DArray(coralInfoSub.get());
        coralInfo = gson.fromJson(coralInfoSub.get(), boolean[][].class);
    }
}