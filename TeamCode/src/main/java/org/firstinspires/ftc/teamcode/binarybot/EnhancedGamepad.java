package org.firstinspires.ftc.teamcode.binarybot;

import com.qualcomm.robotcore.hardware.Gamepad;


public class EnhancedGamepad {

    public final int BUTTONS = 14;
    public final int AXES = 6;

    public Gamepad gamepad;

    boolean[] pressed;
    boolean[] prev_poll;
    float[] axes;

    public EnhancedGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.prev_poll = new boolean[BUTTONS];
        this.pressed = new boolean[BUTTONS];
        this.axes = new float[AXES];
    }

    public boolean pressed(Button button) {
        return pressed[button.ordinal()];
    }

    public boolean released(Button button) {
        return !pressed[button.ordinal()];
    }

    public boolean justPressed(Button button) {
        return !prev_poll[button.ordinal()] && this.pressed[button.ordinal()];
    }

    public boolean justReleased(Button button) {
        return prev_poll[button.ordinal()] && !this.pressed[button.ordinal()];
    }

    public void poll() {
        this.prev_poll = this.pressed;
        this.pressed = new boolean[BUTTONS];

        pressed[Button.A.ordinal()] = gamepad.a;
        pressed[Button.B.ordinal()] = gamepad.b;
        pressed[Button.X.ordinal()] = gamepad.x;
        pressed[Button.Y.ordinal()] = gamepad.y;
        pressed[Button.DPAD_UP.ordinal()] = gamepad.dpad_up;
        pressed[Button.DPAD_DOWN.ordinal()] = gamepad.dpad_down;
        pressed[Button.DPAD_LEFT.ordinal()] = gamepad.dpad_left;
        pressed[Button.DPAD_RIGHT.ordinal()] = gamepad.dpad_right;
        pressed[Button.LEFT_BUMPER.ordinal()] = gamepad.left_bumper;
        pressed[Button.RIGHT_BUMPER.ordinal()] = gamepad.right_bumper;
        pressed[Button.LEFT_STICK.ordinal()] = gamepad.left_stick_button;
        pressed[Button.RIGHT_STICK.ordinal()] = gamepad.right_stick_button;
        pressed[Button.START.ordinal()] = gamepad.start;
        pressed[Button.BACK.ordinal()] = gamepad.back;

        axes[Axis.LEFT_STICK_X.ordinal()] = gamepad.left_stick_x;
        axes[Axis.LEFT_STICK_Y.ordinal()] = gamepad.left_stick_y;
        axes[Axis.RIGHT_STICK_X.ordinal()] = gamepad.right_stick_x;
        axes[Axis.RIGHT_STICK_Y.ordinal()] = gamepad.right_stick_y;
        axes[Axis.LEFT_TRIGGER.ordinal()] = gamepad.left_trigger;
        axes[Axis.RIGHT_TRIGGER.ordinal()] = gamepad.right_trigger;
    }


    public enum Button {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_STICK, RIGHT_STICK,
        START,
        BACK;
    }

    public enum Axis {
        LEFT_STICK_X, LEFT_STICK_Y,
        RIGHT_STICK_X, RIGHT_STICK_Y,
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

}
