package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class CustomController {
    private final GamepadEx gamepadEx;

    public CustomController(GamepadEx gamepadEx) {
        this.gamepadEx = gamepadEx;
    }

    public ControllerCondition onButtonPressed(GamepadKeys.Button button) {
        return new ControllerCondition(this, new Trigger(() -> gamepadEx.getButton(button)));
    }

    public ControllerCondition onAPressed() {
        return onButtonPressed(GamepadKeys.Button.A);
    }

    public ControllerCondition onBPressed() {
        return onButtonPressed(GamepadKeys.Button.B);
    }

    public ControllerCondition onXPressed() {
        return onButtonPressed(GamepadKeys.Button.X);
    }

    public ControllerCondition onYPressed() {
        return onButtonPressed(GamepadKeys.Button.Y);
    }

    public ControllerCondition onBackPressed() {
        return onButtonPressed(GamepadKeys.Button.BACK);
    }

    public ControllerCondition onLeftPressed() {
        return onButtonPressed(GamepadKeys.Button.DPAD_LEFT);
    }

    public ControllerCondition onRightPressed() {
        return onButtonPressed(GamepadKeys.Button.DPAD_RIGHT);
    }

    public ControllerCondition onUpPressed() {
        return onButtonPressed(GamepadKeys.Button.DPAD_UP);
    }

    public ControllerCondition onDownPressed() {
        return onButtonPressed(GamepadKeys.Button.DPAD_DOWN);
    }

    public ControllerCondition onRightBumperPressed() {
        return onButtonPressed(GamepadKeys.Button.RIGHT_BUMPER);
    }

    public ControllerCondition onLeftBumperPressed() {
        return onButtonPressed(GamepadKeys.Button.LEFT_BUMPER);
    }
}
