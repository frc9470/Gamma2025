package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    /**
     * Motor IDs
     */
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(14, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(15, "rio");

    public static final CanDeviceId ALGAE_ROLLER = new CanDeviceId(17, "rio");
    public static final CanDeviceId ALGAE_PIVOT = new CanDeviceId(18, "rio");

    public static final CanDeviceId CORAL_INTAKE = new CanDeviceId(16, "rio");
    public static final CanDeviceId FUNNEL = new CanDeviceId(19, "rio");

    /**
     * Beam Break IDs
     */
    public static final int CORAL_BREAK = 1;
}
