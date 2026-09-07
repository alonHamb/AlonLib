package alonlib.commands.button

import alonlib.gamepad.GamepadEx
import alonlib.gamepad.GamepadKeys

/** A [Button] backed by one or more [alonlib.gamepad.GamepadEx] buttons -- active only while all of [buttons] are held. */
class GamepadButton(private val gamepad: GamepadEx, private vararg val buttons: GamepadKeys.Button) : Button() {
    override fun get() = buttons.all { gamepad.getButton(it) }
}
