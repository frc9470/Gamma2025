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

    public ButtonBoardPub(Telemetry logger) {
        this.logger = logger;

        // buttonBoard = new Joystick(1);

        // TODO: will actually be populated by website!
        corals = new boolean[4][12];

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 12; j++) {
                corals[i][j] = false;
            }
        }

        // branchId = 2 (NO SUBTRACT), level = 4 (- 1)
        // corals[0][2] = true;
        // corals[1][2] = true;
        // corals[2][2] = true;
        // corals[3][2] = true;
    }

    @Override
    public void periodic() {
//         int opBranchId = 0;
//         int level = 0;

//         corals[level][opBranchId] = true;
//         // <get some updated

//         // TODO: actually populate with website!
// //        logger.coralInfo.set(Util.Boolean2DArrayToJSON(corals));

//         String coralString = gson.toJson(corals);

//         logger.coralInfo.set(coralString);
    }
}