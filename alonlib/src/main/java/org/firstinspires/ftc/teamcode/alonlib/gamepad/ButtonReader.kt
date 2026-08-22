package org.firstinspires.ftc.teamcode.alonlib.gamepad

/** Edge-detects a button, either read from [gamepad]/[button] or an arbitrary [buttonState] supplier. */
open class ButtonReader(private val buttonState: () -> Boolean) : KeyReader {

    constructor(gamepad: GamepadEx, button: GamepadKeys.Button) : this({ gamepad.getButton(button) })

    private var currState = buttonState()
    private var lastState = currState

    override fun readValue() {
        lastState = currState
        currState = buttonState()
    }

    override fun isDown() = buttonState()
    override fun wasJustPressed() = !lastState && currState
    override fun wasJustReleased() = lastState && !currState
    override fun stateJustChanged() = lastState != currState
}
