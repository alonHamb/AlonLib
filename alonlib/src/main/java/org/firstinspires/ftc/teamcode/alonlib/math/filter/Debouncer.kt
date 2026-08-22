package org.firstinspires.ftc.teamcode.alonlib.math.filter

/**
 * Requires a boolean input to hold steady away from its baseline for [debounceTimeSeconds] before
 * the debounced output follows it -- filters out brief flickers (e.g. a bouncy limit switch).
 */
class Debouncer(var debounceTimeSeconds: Double, var type: DebounceType = DebounceType.RISING) {

    enum class DebounceType { RISING, FALLING, BOTH }

    private var baseline = when (type) {
        DebounceType.BOTH, DebounceType.RISING -> false
        DebounceType.FALLING                    -> true
    }

    private var prevTime = System.nanoTime() / 1e9

    private fun resetTimer() {
        prevTime = System.nanoTime() / 1e9
    }

    private fun hasElapsed() = System.nanoTime() / 1e9 - prevTime >= debounceTimeSeconds

    fun calculate(input: Boolean): Boolean {
        if (input == baseline) resetTimer()

        return if (hasElapsed()) {
            if (type == DebounceType.BOTH) {
                baseline = input
                resetTimer()
            }
            input
        } else {
            baseline
        }
    }
}
