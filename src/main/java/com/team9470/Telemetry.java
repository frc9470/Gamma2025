package com.team9470;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Telemetry {
    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");

    /* For button board communications */
    // public final StringPublisher coralInfo = driveStateTable.getStringTopic("CoralInfo").publish();
    public final IntegerPublisher branch = driveStateTable.getIntegerTopic("branch").publish();
    public final IntegerPublisher level = driveStateTable.getIntegerTopic("level").publish();
    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
}
