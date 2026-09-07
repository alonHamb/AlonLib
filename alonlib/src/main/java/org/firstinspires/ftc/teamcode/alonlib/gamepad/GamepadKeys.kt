package org.firstinspires.ftc.teamcode.alonlib.gamepad

/** Every button/trigger [GamepadEx] can read. */
object GamepadKeys {

    enum class Button {
        Y, X, A, B, LEFT_BUMPER, RIGHT_BUMPER, BACK,
        START, OPTIONS, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        TRIANGLE, SQUARE, CROSS, CIRCLE,
        PS, SHARE, TOUCHPAD, TOUCHPAD_FINGER_1, TOUCHPAD_FINGER_2,
    }

    enum class Trigger {
        LEFT_TRIGGER, RIGHT_TRIGGER,
    }
}
