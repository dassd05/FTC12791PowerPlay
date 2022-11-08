package org.firstinspires.ftc.teamcode.gamepad;

import java.util.EventListener;

public abstract class GamepadListener implements EventListener {

    public enum Button {
        a, b, x, y,
        dpad_up, dpad_down, dpad_left, dpad_right,
        left_bumper, right_bumper,
        left_stick_button, right_stick_button,
        guide, start, back
    }

    public void onLeftStickMove(float x, float y, float dx, float dy) {}
    public void onRightStickMove(float x, float y, float dx, float dy) {}
    public void onLeftTriggerMove(float x, float dx) {}
    public void onRightTriggerMove(float x, float dx) {}
    public void onButtonPress(Button button) {}
    public void onButtonRelease(Button button) {}

//    circle = b;
//    cross = a;
//    triangle = y;
//    square = x;
//    share = back;
//    options = start;
//    ps = guide;
}