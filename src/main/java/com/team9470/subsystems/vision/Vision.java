package com.team9470.subsystems.vision;

import com.team9470.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import static com.team9470.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private static Vision instance;

    private final VisionDevice frontL = new VisionDevice("OV2311-L", VisionConstants.FRONT_LEFT_CAMERA_OFFSET);
    private final VisionDevice frontR = new VisionDevice("OV2311-R", VisionConstants.FRONT_RIGHT_CAMERA_OFFSET);
    //private final VisionDevice back = new VisionDevice("back", VisionConstants.BACK_CAMERA_OFFSET);
    private boolean visionDisabled = false;

    private final List<VisionDevice> devices = List.of(frontL, frontR);

    private Vision() {
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    int count = 0;
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vision/heartbeat", count);
        count++;
        devices.forEach(visionDevice -> visionDevice.updatePosition(Swerve.getInstance()));
    }

    public boolean isVisionDisabled() {
        return visionDisabled;
    }

    public void setVisionDisabled(boolean visionDisabled) {
        this.visionDisabled = visionDisabled;
    }
}