package org.firstinspires.ftc.teamcode.alonlib.gamepad

/** A [ButtonReader] that flips a persistent on/off [state] each time the button is released. */
class ToggleButtonReader(buttonState: () -> Boolean) : ButtonReader(buttonState) {

    constructor(gamepad: GamepadEx, button: GamepadKeys.Button) : this({ gamepad.getButton(button) })

    private var toggleState = false

    val state: Boolean
        get() {
            if (wasJustReleased()) toggleState = !toggleState
            return toggleState
        }
}
