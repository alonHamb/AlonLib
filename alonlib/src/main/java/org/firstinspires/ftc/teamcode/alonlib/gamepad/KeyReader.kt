package org.firstinspires.ftc.teamcode.alonlib.gamepad

/** Edge-detecting read of a boolean control (a button or a thresholded trigger/axis). Call [readValue] once per loop to advance it. */
interface KeyReader {
    fun readValue()
    fun isDown(): Boolean
    fun wasJustPressed(): Boolean
    fun wasJustReleased(): Boolean
    fun stateJustChanged(): Boolean
}
