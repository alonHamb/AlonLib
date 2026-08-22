package org.firstinspires.ftc.teamcode.alonlib.purepursuit.actions

/** An action a [org.firstinspires.ftc.teamcode.alonlib.purepursuit.Path] fires once [isTriggered] becomes true (e.g. "robot crossed Y=2m"). */
abstract class TriggeredAction {

    private var alreadyPerformed = false

    /** Polled every [org.firstinspires.ftc.teamcode.alonlib.purepursuit.Path.loop] call; fires [doAction] once [isTriggered] is true. */
    fun loop() {
        if (isTriggered()) {
            doAction(alreadyPerformed)
            alreadyPerformed = true
        }
    }

    fun reset() {
        alreadyPerformed = false
    }

    abstract fun isTriggered(): Boolean

    /** [alreadyPerformed] is true if this has already fired once before (e.g. to only act on the first crossing). */
    abstract fun doAction(alreadyPerformed: Boolean)
}
