package org.firstinspires.ftc.teamcode.subsystem.io;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public HardwareMap hardwareMap;
    public Servo left;
    public Servo right;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        left = hardwareMap.get(Servo.class, "armLeft");
        right = hardwareMap.get(Servo.class, "armRight");
    }
}
