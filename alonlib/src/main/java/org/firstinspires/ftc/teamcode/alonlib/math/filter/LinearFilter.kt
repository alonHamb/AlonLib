package org.firstinspires.ftc.teamcode.alonlib.math.filter

import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import kotlin.math.exp

/**
 * A general linear FIR/IIR digital filter:
 * `y[n] = (b0*x[n] + b1*x[n-1] + ... + bP*x[n-P]) - (a0*y[n-1] + a1*y[n-2] + ... + aQ*y[n-Q])`
 *
 * [calculate] must be called on a known, regular period -- filter gains are inherently a function
 * of sample rate. Use the companion factories ([singlePoleIIR], [highPass], [movingAverage],
 * [finiteDifference]) rather than the raw gains constructor unless you're hand-deriving your own
 * filter.
 */
class LinearFilter(feedforwardGains: DoubleArray, feedbackGains: DoubleArray) {

    private val inputs = DoubleCircularBuffer(feedforwardGains.size)
    private val outputs = DoubleCircularBuffer(feedbackGains.size)
    private val feedforwardGains = feedforwardGains.copyOf()
    private val feedbackGains = feedbackGains.copyOf()

    var lastValue = 0.0
        private set

    fun reset() {
        inputs.clear()
        outputs.clear()
    }

    /** Resets the filter, seeding its input/output history with [inputBuffer]/[outputBuffer] (most-recent-first). */
    fun reset(inputBuffer: DoubleArray, outputBuffer: DoubleArray) {
        reset()
        if (inputBuffer.size != feedforwardGains.size || outputBuffer.size != feedbackGains.size) {
            robotPrintError("reset() buffer sizes don't match this filter's gains")
            return
        }
        for (input in inputBuffer) inputs.addFirst(input)
        for (output in outputBuffer) outputs.addFirst(output)
    }

    fun calculate(input: Double): Double {
        var result = 0.0

        if (feedforwardGains.isNotEmpty()) inputs.addFirst(input)

        for (i in feedforwardGains.indices) result += inputs[i] * feedforwardGains[i]
        for (i in feedbackGains.indices) result -= outputs[i] * feedbackGains[i]

        if (feedbackGains.isNotEmpty()) outputs.addFirst(result)

        lastValue = result
        return result
    }

    companion object {
        /**
         * A one-pole IIR low-pass filter: `y[n] = (1-gain)*x[n] + gain*y[n-1]` where
         * `gain = e^(-period/timeConstant)`. Stable for any positive [timeConstant].
         *
         * `timeConstant = 1 / (2*pi*f)` where `f` is the cutoff frequency in Hz (frequencies
         * above it get attenuated).
         */
        fun singlePoleIIR(timeConstant: Double, period: Double): LinearFilter {
            val gain = exp(-period / timeConstant)
            return LinearFilter(doubleArrayOf(1.0 - gain), doubleArrayOf(-gain))
        }

        /**
         * A first-order high-pass filter: `y[n] = gain*x[n] - gain*x[n-1] + gain*y[n-1]` where
         * `gain = e^(-period/timeConstant)`. Stable for any positive [timeConstant].
         *
         * `timeConstant = 1 / (2*pi*f)` where `f` is the cutoff frequency in Hz (frequencies
         * below it get attenuated).
         */
        fun highPass(timeConstant: Double, period: Double): LinearFilter {
            val gain = exp(-period / timeConstant)
            return LinearFilter(doubleArrayOf(gain, -gain), doubleArrayOf(-gain))
        }

        /** A [taps]-tap FIR moving average: `y[n] = (x[taps-1] + ... + x[0]) / taps`. Always stable. */
        fun movingAverage(taps: Int): LinearFilter {
            if (taps <= 0) {
                robotPrintError("movingAverage needs at least 1 tap")
                return LinearFilter(doubleArrayOf(1.0), DoubleArray(0))
            }
            return LinearFilter(DoubleArray(taps) { 1.0 / taps }, DoubleArray(0))
        }

        /**
         * A finite-difference filter approximating the [derivative]-th derivative of the input,
         * sampled at the given [stencil] points (0 = current sample, -1 = previous, etc. -- avoid
         * positive stencil points for real-time/streaming use, since those need future samples).
         */
        fun finiteDifference(derivative: Int, stencil: IntArray, period: Double): LinearFilter {
            if (derivative < 1) {
                robotPrintError("order of derivative must be >= 1")
                return LinearFilter(doubleArrayOf(1.0), DoubleArray(0))
            }
            val samples = stencil.size
            if (samples <= 0) {
                robotPrintError("number of samples must be > 0")
                return LinearFilter(doubleArrayOf(1.0), DoubleArray(0))
            }
            if (derivative >= samples) {
                robotPrintError("order of derivative must be less than number of samples")
                return LinearFilter(doubleArrayOf(1.0), DoubleArray(0))
            }

            // See https://en.wikipedia.org/wiki/Finite_difference_coefficient#Arbitrary_stencil_points --
            // solve S*a = d!*e_derivative for the FIR coefficients a.
            val s = Matrix(samples, samples)
            for (row in 0 until samples) {
                for (col in 0 until samples) {
                    s[row, col] = Math.pow(stencil[col].toDouble(), row.toDouble())
                }
            }

            val d = Matrix(samples, 1)
            d[derivative, 0] = factorial(derivative).toDouble()

            val a = s.solve(d) / Math.pow(period, derivative.toDouble())

            val ffGains = DoubleArray(samples) { a[samples - it - 1, 0] }
            return LinearFilter(ffGains, DoubleArray(0))
        }

        /** A backward finite-difference filter using [samples] consecutive past+current samples. */
        fun backwardFiniteDifference(derivative: Int, samples: Int, period: Double): LinearFilter {
            val stencil = IntArray(samples) { -(samples - 1) + it }
            return finiteDifference(derivative, stencil, period)
        }

        private fun factorial(n: Int): Int = if (n < 2) 1 else n * factorial(n - 1)
    }
}
