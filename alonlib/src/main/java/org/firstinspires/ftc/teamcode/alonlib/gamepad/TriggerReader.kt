package org.firstinspires.ftc.teamcode.alonlib.gamepad

/** Edge-detects an analog [trigger], treating it as "down" past [threshold]. */
class TriggerReader(private val gamepad: GamepadEx, private val trigger: GamepadKeys.Trigger, private val threshold: Double = 0.5) : KeyReader {

    private var currState = gamepad.getTrigger(trigger) > threshold
    private var lastState = currState

    override fun readValue() {
        lastState = currState
        currState = gamepad.getTrigger(trigger) > threshold
    }

    override fun isDown() = currState
    override fun wasJustPressed() = !lastState && currState
    override fun wasJustReleased() = lastState && !currState
    override fun stateJustChanged() = lastState != currState
}
