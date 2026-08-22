package org.firstinspires.ftc.teamcode.alonlib.math.filter

/**
 * A [Debouncer] for arbitrary types, not just [Boolean] -- e.g. a color sensor's detected color,
 * or which AprilTag is currently visible. Requires [input] to hold steady (by [equals]) for
 * [debounceMillis] before [state] follows it.
 */
class GenericDebouncer<T>(var debounceMillis: Double, initial: T) {

    var state: T = initial
        private set

    private var lastInput: T = initial
    private var previousTime = 0L

    fun calculate(input: T): T {
        if (input == lastInput) {
            lastInput = input
            previousTime = System.nanoTime()
        }

        if (System.nanoTime() - previousTime >= debounceMillis * 1e6) {
            state = input
        }

        return state
    }

    /** Resets [state] (and the debounce timer) to [newState] immediately -- for when you know it's changed and don't want to wait. */
    fun reset(newState: T) {
        previousTime = 0L
        state = newState
        lastInput = newState
    }
}
