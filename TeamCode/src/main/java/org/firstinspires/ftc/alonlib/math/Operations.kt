package org.firstinspires.ftc.teamcode.alonlib.math

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import kotlin.math.abs

/**
 * If the absolute value is smaller than the deadband, the value becomes 0.
 * Otherwise, it stays the same.
 *
 * - [deadband] must be positive. [value] can be anything.
 */
fun simpleDeadband(value: Double, deadband: Double): Double {
    if (deadband < 0.0) {
        robotPrintError("deadband is negative")
        return value
    }
    return if (abs(value) >= deadband) value else 0.0
}

/**
 * If the absolute value is smaller than the deadband, it becomes 0.
 * Otherwise, it is mapped from a range where deadband is the minimum and 1 is the maximum,
 * to a range where zero is the minimum and 1 is the maximum. (Or, if the value is negative,
 * from a range where -1 is the minimum and -deadband is the maximum, to a range where -1 is
 * the minimum and zero is the maximum.)
 * - [deadband] must be between 0 and 1. [value] must be between -1 and 1.
 *
 * Some examples:
 * - continuousDeadband(0.05, 0.1) will return 0.0.
 * - continuousDeadband(0.1, 0.1) will return 0.0.
 * - continuousDeadband(0.5, 0.1) will return 0.44444
 * - continuousDeadband(1, 0.1) will return 1.0.
 */
fun continuousDeadband(value: Double, deadband: Double): Double {
    if (deadband !in 0.0..1.0) {
        robotPrintError("deadband is out of bounds: $deadband")
        return value
    }
    if (value !in -1.0..1.0) {
        robotPrintError("value is out of bounds: $value")
        return value
    }

    return if (value > deadband) {
        mapRange(value, deadband, 1.0, 0.0, 1.0)
    } else if (value < -deadband) {
        mapRange(value, -1.0, -deadband, -1.0, 0.0)
    } else {
        0.0
    }
}

fun clamp(value: Double, min: Double, max: Double): Double {
    return if (min > max) 0.0 else clamp(value, min, max)
}

/**
 * Gets a start range defined by [startMin] and [startMax] and an end range defined by [endMin] and [endMax], and a
 * value that is relative to the first range.
 *
 * @return The value relative to the end range.
 */
fun mapRange(
    value: Double,
    startMin: Double,
    startMax: Double,
    endMin: Double,
    endMax: Double
): Double {
    if (startMin >= startMax) {
        robotPrintError("startMin is equal/bigger than starMax")
        return value
    }
    if (endMin >= endMax) {
        robotPrintError("endMin is equal/bigger than endMax")
        return value
    }
    return endMin + (endMax - endMin) / (startMax - startMin) * (value - startMin)
}

/**
 * Gets a start range defined by [startMin] and [startMax] and an end range defined by [endMin] and [endMax], and a
 * value that is relative to the first range.
 *
 * @return The value relative to the end range.
 */
fun mapRange(value: Int, startMin: Int, startMax: Int, endMin: Int, endMax: Int): Int {
    return mapRange(
        value.toDouble(),
        startMin.toDouble(),
        startMax.toDouble(),
        endMin.toDouble(),
        endMax.toDouble()
    ).toInt()
}

fun median(collection: Collection<Double>): Double {
    return median(collection.toDoubleArray())
}

fun median(array: Array<Double>): Double {
    return median(array.toDoubleArray())
}

fun median(array: DoubleArray): Double {
    val sortedArray = array.sorted()
    val size = sortedArray.size
    if (size == 2) return sortedArray.average()

    return if (size % 2 == 0) {
        (sortedArray[size / 2] + sortedArray[(size / 2) - 1]) / 2.0
    } else {
        array[(size / 2)]
    }
}
