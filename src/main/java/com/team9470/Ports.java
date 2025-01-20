package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(13, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(14, "rio");

    public static final CanDeviceId ALGAE_ROLLER = new CanDeviceId(10, "rio");
    public static final CanDeviceId ALGAE_PIVOT = new CanDeviceId(11, "rio");

    public static final CanDeviceId CORAL_INTAKE = new CanDeviceId(12, "rio");

    /* Beam Break ids */
    public static final int CORAL_BREAK = 0;
}
