package org.firstinspires.ftc.teamcode.alonlib.math.estimator

import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix

/**
 * Incorporates time-delayed measurements (e.g. from vision, which typically arrives a frame or
 * two late) into a [KalmanTypeFilter]'s state estimate: rewinds the filter to its state as of the
 * measurement's timestamp, applies the correction there, then replays the inputs/local
 * measurements recorded since to catch back up to the present.
 */
class KalmanFilterLatencyCompensator {

    private class ObserverSnapshot(observer: KalmanTypeFilter, val inputs: Matrix, val localMeasurements: Matrix) {
        val xHat: Matrix = observer.xHat
        val errorCovariances: Matrix = observer.p
    }

    private val pastObserverSnapshots = mutableListOf<Pair<Double, ObserverSnapshot>>()

    /** Clears the observer snapshot buffer. */
    fun reset() {
        pastObserverSnapshots.clear()
    }

    /** Records [observer]'s state at [timestampSeconds], for later replay in [applyPastGlobalMeasurement]. */
    fun addObserverState(observer: KalmanTypeFilter, u: Matrix, localY: Matrix, timestampSeconds: Double) {
        pastObserverSnapshots.add(timestampSeconds to ObserverSnapshot(observer, u, localY))
        if (pastObserverSnapshots.size > MAX_PAST_OBSERVER_STATES) pastObserverSnapshots.removeAt(0)
    }

    /**
     * Applies a time-delayed global measurement (e.g. from vision) taken at [timestampSeconds]:
     * rewinds [observer] to its recorded state as of that time, applies [globalMeasurementCorrect]
     * there, then replays every recorded input/local-measurement since to catch back up.
     */
    fun applyPastGlobalMeasurement(
        observer: KalmanTypeFilter,
        nominalDtSeconds: Double,
        y: Matrix,
        globalMeasurementCorrect: (Matrix, Matrix) -> Unit,
        timestampSeconds: Double,
    ) {
        if (pastObserverSnapshots.isEmpty()) return

        val maxIdx = pastObserverSnapshots.size - 1
        var low = 0
        var high = maxIdx

        // Binary search for the first snapshot at or after the measurement's timestamp.
        while (low != high) {
            val mid = (low + high) / 2
            if (pastObserverSnapshots[mid].first < timestampSeconds) low = mid + 1 else high = mid
        }

        val indexOfClosestEntry: Int
        if (low == 0) {
            if (timestampSeconds < pastObserverSnapshots[low].first) return
            indexOfClosestEntry = 0
        } else if (low == maxIdx && pastObserverSnapshots[low].first < timestampSeconds) {
            indexOfClosestEntry = maxIdx
        } else {
            val nextIdx = low
            val prevIdx = nextIdx - 1
            val prevTimeDiff = kotlin.math.abs(timestampSeconds - pastObserverSnapshots[prevIdx].first)
            val nextTimeDiff = kotlin.math.abs(timestampSeconds - pastObserverSnapshots[nextIdx].first)
            indexOfClosestEntry = if (prevTimeDiff <= nextTimeDiff) prevIdx else nextIdx
        }

        var lastTimestamp = pastObserverSnapshots[indexOfClosestEntry].first - nominalDtSeconds

        for (i in indexOfClosestEntry until pastObserverSnapshots.size) {
            val (key, snapshot) = pastObserverSnapshots[i]

            if (i == indexOfClosestEntry) {
                observer.p = snapshot.errorCovariances
                observer.xHat = snapshot.xHat
            }

            observer.predict(snapshot.inputs, key - lastTimestamp)
            observer.correct(snapshot.inputs, snapshot.localMeasurements)

            if (i == indexOfClosestEntry) {
                // The measurement's timestamp is close to but probably not exactly this snapshot's --
                // assumes the gap is small enough to ignore.
                globalMeasurementCorrect(snapshot.inputs, y)
            }
            lastTimestamp = key

            pastObserverSnapshots[i] = key to ObserverSnapshot(observer, snapshot.inputs, snapshot.localMeasurements)
        }
    }

    companion object {
        private const val MAX_PAST_OBSERVER_STATES = 300
    }
}
