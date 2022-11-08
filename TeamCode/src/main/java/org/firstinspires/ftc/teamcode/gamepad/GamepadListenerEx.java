package org.firstinspires.ftc.teamcode.gamepad;

import androidx.annotation.CallSuper;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashSet;
import java.util.Set;

public class GamepadListenerEx extends GamepadListener {

    protected Gamepad gamepad;
    protected Set<Button> pressedButtons = new HashSet<>();

    private float lastLeftStickX;
    private float lastLeftStickY;
    private float lastRightStickX;
    private float lastRightStickY;
    private float lastLeftTrigger;
    private float lastRightTrigger;

    public GamepadListenerEx(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    @Override @CallSuper
    public void onLeftStickMove(float x, float y, float dx, float dy) {
        lastLeftStickX = x;
        lastLeftStickY = y;
    }
    @Override @CallSuper
    public void onRightStickMove(float x, float y, float dx, float dy) {
        lastRightStickX = x;
        lastRightStickY = y;
    }
    @Override @CallSuper
    public void onLeftTriggerMove(float x, float dx) {
        lastLeftTrigger = x;
    }
    @Override @CallSuper
    public void onRightTriggerMove(float x, float dx) {
        lastRightTrigger = x;
    }
    @Override @CallSuper
    public void onButtonPress(Button button) {
        pressedButtons.add(button);
    }
    @Override @CallSuper
    public void onButtonRelease(Button button) {
        pressedButtons.remove(button);
    }

    public final boolean isButtonPressed(Button button) {
        return pressedButtons.contains(button);
    }

    public void update() {
        // the movement stuff (sticks / triggers)
        if (gamepad.left_stick_x != lastLeftStickX || gamepad.left_stick_y != lastLeftStickY)
            onLeftStickMove(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.left_stick_x - lastLeftStickX, gamepad.left_stick_y - lastLeftStickY);
        if (gamepad.right_stick_x != lastRightStickX || gamepad.right_stick_y != lastRightStickY)
            onRightStickMove(gamepad.right_stick_x, gamepad.right_stick_y, gamepad.right_stick_x - lastRightStickX, gamepad.right_stick_y - lastRightStickY);
        if (gamepad.left_trigger != lastLeftTrigger)
            onLeftTriggerMove(gamepad.left_trigger, gamepad.left_trigger - lastLeftTrigger);
        if (gamepad.right_trigger != lastRightTrigger)
            onRightTriggerMove(gamepad.right_trigger, gamepad.right_trigger - lastRightTrigger);

        // buttons press
        if (gamepad.a && !pressedButtons.contains(Button.a))                                   onButtonPress(Button.a);
        if (gamepad.b && !pressedButtons.contains(Button.b))                                   onButtonPress(Button.b);
        if (gamepad.x && !pressedButtons.contains(Button.x))                                   onButtonPress(Button.x);
        if (gamepad.y && !pressedButtons.contains(Button.y))                                   onButtonPress(Button.y);
        if (gamepad.dpad_up && !pressedButtons.contains(Button.dpad_up))                       onButtonPress(Button.dpad_up);
        if (gamepad.dpad_down && !pressedButtons.contains(Button.dpad_down))                   onButtonPress(Button.dpad_down);
        if (gamepad.dpad_left && !pressedButtons.contains(Button.dpad_left))                   onButtonPress(Button.dpad_left);
        if (gamepad.dpad_right && !pressedButtons.contains(Button.dpad_right))                 onButtonPress(Button.dpad_right);
        if (gamepad.left_bumper && !pressedButtons.contains(Button.left_bumper))               onButtonPress(Button.left_bumper);
        if (gamepad.right_bumper && !pressedButtons.contains(Button.right_bumper))             onButtonPress(Button.right_bumper);
        if (gamepad.left_stick_button && !pressedButtons.contains(Button.left_stick_button))   onButtonPress(Button.left_stick_button);
        if (gamepad.right_stick_button && !pressedButtons.contains(Button.right_stick_button)) onButtonPress(Button.right_stick_button);
        if (gamepad.guide && !pressedButtons.contains(Button.guide))                           onButtonPress(Button.guide);
        if (gamepad.start && !pressedButtons.contains(Button.start))                           onButtonPress(Button.start);
        if (gamepad.back && !pressedButtons.contains(Button.back))                             onButtonPress(Button.back);
        // buttons release
        if (!gamepad.a && pressedButtons.contains(Button.a))                                   onButtonRelease(Button.a);
        if (!gamepad.b && pressedButtons.contains(Button.b))                                   onButtonRelease(Button.b);
        if (!gamepad.x && pressedButtons.contains(Button.x))                                   onButtonRelease(Button.x);
        if (!gamepad.y && pressedButtons.contains(Button.y))                                   onButtonRelease(Button.y);
        if (!gamepad.dpad_up && pressedButtons.contains(Button.dpad_up))                       onButtonRelease(Button.dpad_up);
        if (!gamepad.dpad_down && pressedButtons.contains(Button.dpad_down))                   onButtonRelease(Button.dpad_down);
        if (!gamepad.dpad_left && pressedButtons.contains(Button.dpad_left))                   onButtonRelease(Button.dpad_left);
        if (!gamepad.dpad_right && pressedButtons.contains(Button.dpad_right))                 onButtonRelease(Button.dpad_right);
        if (!gamepad.left_bumper && pressedButtons.contains(Button.left_bumper))               onButtonRelease(Button.left_bumper);
        if (!gamepad.right_bumper && pressedButtons.contains(Button.right_bumper))             onButtonRelease(Button.right_bumper);
        if (!gamepad.left_stick_button && pressedButtons.contains(Button.left_stick_button))   onButtonRelease(Button.left_stick_button);
        if (!gamepad.right_stick_button && pressedButtons.contains(Button.right_stick_button)) onButtonRelease(Button.right_stick_button);
        if (!gamepad.guide && pressedButtons.contains(Button.guide))                           onButtonRelease(Button.guide);
        if (!gamepad.start && pressedButtons.contains(Button.start))                           onButtonRelease(Button.start);
        if (!gamepad.back && pressedButtons.contains(Button.back))                             onButtonRelease(Button.back);

    }
}
