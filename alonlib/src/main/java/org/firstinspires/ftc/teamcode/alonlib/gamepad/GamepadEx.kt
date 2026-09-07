package org.firstinspires.ftc.teamcode.alonlib.gamepad

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.alonlib.commands.button.GamepadButton
import org.firstinspires.ftc.teamcode.alonlib.math.filter.SlewRateLimiter

/** A richer wrapper over the raw FTC SDK [Gamepad]: edge-detected buttons, optional joystick slew-rate limiting, and [GamepadButton]s for binding commands. */
class GamepadEx(val gamepad: Gamepad) {

    private val buttonReaders = GamepadKeys.Button.entries.associateWith { ButtonReader(this, it) }
    private val gamepadButtons = GamepadKeys.Button.entries.associateWith { GamepadButton(this, it) }

    private var lxLimiter: SlewRateLimiter? = null
    private var lyLimiter: SlewRateLimiter? = null
    private var rxLimiter: SlewRateLimiter? = null
    private var ryLimiter: SlewRateLimiter? = null

    fun getButton(button: GamepadKeys.Button): Boolean = when (button) {
        GamepadKeys.Button.A, GamepadKeys.Button.CROSS -> gamepad.a
        GamepadKeys.Button.B, GamepadKeys.Button.CIRCLE -> gamepad.b
        GamepadKeys.Button.X, GamepadKeys.Button.SQUARE -> gamepad.x
        GamepadKeys.Button.Y, GamepadKeys.Button.TRIANGLE -> gamepad.y
        GamepadKeys.Button.LEFT_BUMPER -> gamepad.left_bumper
        GamepadKeys.Button.RIGHT_BUMPER -> gamepad.right_bumper
        GamepadKeys.Button.DPAD_UP -> gamepad.dpad_up
        GamepadKeys.Button.DPAD_DOWN -> gamepad.dpad_down
        GamepadKeys.Button.DPAD_LEFT -> gamepad.dpad_left
        GamepadKeys.Button.DPAD_RIGHT -> gamepad.dpad_right
        GamepadKeys.Button.BACK -> gamepad.back
        GamepadKeys.Button.START -> gamepad.start
        GamepadKeys.Button.OPTIONS -> gamepad.options
        GamepadKeys.Button.LEFT_STICK_BUTTON -> gamepad.left_stick_button
        GamepadKeys.Button.RIGHT_STICK_BUTTON -> gamepad.right_stick_button
        GamepadKeys.Button.PS -> gamepad.ps
        GamepadKeys.Button.SHARE -> gamepad.share
        GamepadKeys.Button.TOUCHPAD -> gamepad.touchpad
        GamepadKeys.Button.TOUCHPAD_FINGER_1 -> gamepad.touchpad_finger_1
        GamepadKeys.Button.TOUCHPAD_FINGER_2 -> gamepad.touchpad_finger_2
    }

    fun getTrigger(trigger: GamepadKeys.Trigger): Double = when (trigger) {
        GamepadKeys.Trigger.LEFT_TRIGGER -> gamepad.left_trigger.toDouble()
        GamepadKeys.Trigger.RIGHT_TRIGGER -> gamepad.right_trigger.toDouble()
    }

    /** Enables slew-rate limiting on the joysticks (left X/Y, right X/Y) -- pass null for any axis that shouldn't be limited. */
    fun setJoystickSlewRateLimiters(lx: SlewRateLimiter?, ly: SlewRateLimiter?, rx: SlewRateLimiter?, ry: SlewRateLimiter?) = apply {
        lxLimiter = lx
        lyLimiter = ly
        rxLimiter = rx
        ryLimiter = ry
    }

    val leftY: Double get() = lyLimiter?.calculate(-gamepad.left_stick_y.toDouble()) ?: -gamepad.left_stick_y.toDouble()
    val rightY: Double get() = ryLimiter?.calculate(gamepad.right_stick_y.toDouble()) ?: gamepad.right_stick_y.toDouble()
    val leftX: Double get() = lxLimiter?.calculate(gamepad.left_stick_x.toDouble()) ?: gamepad.left_stick_x.toDouble()
    val rightX: Double get() = rxLimiter?.calculate(gamepad.right_stick_x.toDouble()) ?: gamepad.right_stick_x.toDouble()

    fun wasJustPressed(button: GamepadKeys.Button) = buttonReaders.getValue(button).wasJustPressed()
    fun wasJustReleased(button: GamepadKeys.Button) = buttonReaders.getValue(button).wasJustReleased()
    fun isDown(button: GamepadKeys.Button) = buttonReaders.getValue(button).isDown()
    fun stateJustChanged(button: GamepadKeys.Button) = buttonReaders.getValue(button).stateJustChanged()

    /** Advances every button's edge detection -- call this once per loop. */
    fun readButtons() = buttonReaders.values.forEach { it.readValue() }

    fun getGamepadButton(button: GamepadKeys.Button): GamepadButton = gamepadButtons.getValue(button)
}
