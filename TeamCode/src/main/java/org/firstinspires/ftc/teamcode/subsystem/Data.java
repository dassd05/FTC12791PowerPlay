package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Simple class that stores data between opmodes. Basically a centralized
 * database using static values so data can be preserved.
 */
public class Data {

    private Data() {}

    public enum Alliance {
        RED, BLUE, NONE
    }
    public static Alliance alliance = Alliance.NONE;

    public static Pose2d position = new Pose2d(0, 0, 0);

    // we can just migrate this to Constants and it'll be the same
    // but perhaps this way it's clearer
}
