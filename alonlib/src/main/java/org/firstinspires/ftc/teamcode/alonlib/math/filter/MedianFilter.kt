package org.firstinspires.ftc.teamcode.alonlib.math.filter

/**
 * A moving-window median filter, sized [windowSize]. Unlike [LinearFilter]'s moving average, a
 * median is robust to occasional extreme outliers (e.g. a bad vision/LIDAR/ultrasonic reading)
 * instead of averaging them in.
 */
class MedianFilter(private val windowSize: Int) {

    private val valueBuffer = DoubleCircularBuffer(windowSize)
    private val orderedValues = mutableListOf<Double>()

    fun calculate(next: Double): Double {
        var index = java.util.Collections.binarySearch(orderedValues, next)
        if (index < 0) index = -(index + 1)
        orderedValues.add(index, next)

        var size = orderedValues.size
        if (size > windowSize) {
            orderedValues.remove(valueBuffer.removeLast())
            size--
        }

        valueBuffer.addFirst(next)

        return if (size % 2 != 0) {
            orderedValues[size / 2]
        } else {
            (orderedValues[size / 2 - 1] + orderedValues[size / 2]) / 2.0
        }
    }

    fun lastValue() = valueBuffer.getFirst()

    fun reset() {
        orderedValues.clear()
        valueBuffer.clear()
    }
}
