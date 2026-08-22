package org.firstinspires.ftc.teamcode.alonlib.commands

/** Does nothing, for [millis] milliseconds. */
open class WaitCommand(private val millis: Long) : CommandBase() {

    private var startNanos = 0L

    init {
        setName("$name: $millis milliseconds")
    }

    override fun initialize() {
        startNanos = System.nanoTime()
    }

    override fun isFinished() = (System.nanoTime() - startNanos) / 1_000_000 >= millis

    override fun runsWhenDisabled() = true
}
