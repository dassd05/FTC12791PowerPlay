package org.firstinspires.ftc.teamcode.gamepad;

import java.util.EventListener;

/**
 * A listener that picks up gamepad inputs and calls the corresponding method
 * upon the input received. Any listener that uses this needs to extend this class.
 * Updates need to be handled externally.
 */
public abstract class GamepadListener implements EventListener {

    /**
     * All the different types of buttons that the gamepad can supply.
     */
    public enum Button {
        a, b, x, y,
        dpad_up, dpad_down, dpad_left, dpad_right,
        left_bumper, right_bumper,
        left_stick_button, right_stick_button,
        guide, start, back
    }


    /**
     * Called when the gamepad's left joystick moves (ie the x or y position changed).
     * @param x The new x position of the joystick.
     * @param y The new y position of the joystick.
     * @param dx The difference in x values between this and the last invocation of this method.
     * @param dy The difference in y values between this and the last invocation of this method.
     */
    public void onLeftStickMove(float x, float y, float dx, float dy) {}

    /**
     * Called when the gamepad's right joystick moves (ie the x or y position changed).
     * @param x The new x position of the joystick.
     * @param y The new y position of the joystick.
     * @param dx The difference in x values between this and the last invocation of this method.
     * @param dy The difference in y values between this and the last invocation of this method.
     */
    public void onRightStickMove(float x, float y, float dx, float dy) {}

    /**
     * Called when the gamepad's left trigger moves (ie the position changed).
     * @param x The new position of the trigger.
     * @param dx The difference in position between this and the last invocation of this method.
     */
    public void onLeftTriggerMove(float x, float dx) {}

    /**
     * Called when the gamepad's left trigger moves (ie the position changed).
     * @param x The new position of the trigger.
     * @param dx The difference in position between this and the last invocation of this method.
     */
    public void onRightTriggerMove(float x, float dx) {}

    /**
     * Called when any button on the gamepad has been pressed.
     * @param button The {@link Button} that has been pressed.
     */
    public void onButtonPress(Button button) {}

    /**
     * Called when any button on the gamepad has been released.
     * @param button The {@link Button} that has been released.
     */
    public void onButtonRelease(Button button) {}

//    circle = b;
//    cross = a;
//    triangle = y;
//    square = x;
//    share = back;
//    options = start;
//    ps = guide;
}