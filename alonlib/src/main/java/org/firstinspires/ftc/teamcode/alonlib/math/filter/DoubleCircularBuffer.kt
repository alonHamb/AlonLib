package org.firstinspires.ftc.teamcode.alonlib.math.filter

/**
 * A fixed-capacity ring buffer of [Double]s, growing from the front ([addFirst]) -- backs
 * [LinearFilter]/[MedianFilter]'s tap history. Index 0 is always the most recently added element.
 */
class DoubleCircularBuffer(capacity: Int) {

    private val data = DoubleArray(capacity)
    private var front = 0
    var size = 0
        private set

    fun addFirst(value: Double) {
        if (data.isEmpty()) return
        front = (front - 1 + data.size) % data.size
        data[front] = value
        if (size < data.size) size++
    }

    /** @returns the element [index] slots back from the front (0 = most recent), removing nothing. */
    operator fun get(index: Int): Double {
        require(index in 0 until size) { "index $index out of bounds for size $size" }
        return data[(front + index) % data.size]
    }

    /** Removes and returns the oldest element (the back of the buffer). */
    fun removeLast(): Double {
        require(size > 0) { "buffer is empty" }
        val backIndex = (front + size - 1) % data.size
        size--
        return data[backIndex]
    }

    fun getFirst(): Double {
        require(size > 0) { "buffer is empty" }
        return data[front]
    }

    fun clear() {
        front = 0
        size = 0
    }
}
