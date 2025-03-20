package com.team9470.subsystems.sim;

import com.team9470.Telemetry;
import com.team9470.util.Util;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.google.gson.Gson;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBoardPub extends SubsystemBase {
    private static ButtonBoardPub mInstance;
    // Joystick buttonBoard;
    private final boolean[][] corals;
    private final Telemetry logger;
    public Gson gson = new Gson();

    public static ButtonBoardPub getInstance(Telemetry logger) {
        if (mInstance == null) {
            mInstance = new ButtonBoardPub(logger);
        }

        return mInstance;
    }

    private ButtonBoardPub(Telemetry logger) {
        this.logger = logger;

        // buttonBoard = new Joystick(1);

        // TODO: will actually be populated by website!
        corals = new boolean[12][3];

        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 3; j++) {
                corals[i][j] = false;
            }
        }

        corals[0][2] = true;
    }

    @Override
    public void periodic() {
        // TODO: actually populate with website!
//        logger.coralInfo.set(Util.Boolean2DArrayToJSON(corals));

        String coralString = gson.toJson(corals);

        logger.coralInfo.set(coralString);
    }
}