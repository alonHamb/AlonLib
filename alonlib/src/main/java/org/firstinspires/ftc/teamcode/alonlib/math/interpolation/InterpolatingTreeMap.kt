package org.firstinspires.ftc.teamcode.alonlib.math.interpolation

import java.util.TreeMap

/**
 * A sorted [key, value] table that interpolates between its two nearest entries for keys that
 * don't have an exact match, using [keyInterpolator]/[valueInterpolator] to bridge [K]/[V].
 *
 * Unlike [org.firstinspires.ftc.teamcode.alonlib.math.LinearInterpolationTable] (fixed-size,
 * `Double->Double` only), this is mutable ([put] can be called at any time) and works with any
 * [K]/[V] pair -- e.g. a `Rotation2d`-keyed table of `Pose2d`s for trajectory sampling.
 */
open class InterpolatingTreeMap<K : Comparable<K>, V>(
    private val keyInterpolator: InverseInterpolator<K>,
    private val valueInterpolator: Interpolator<V>,
) {
    private val map = TreeMap<K, V>()

    fun put(key: K, value: V) {
        map[key] = value
    }

    fun clear() = map.clear()

    val size get() = map.size
    val isEmpty get() = map.isEmpty()

    /**
     * @returns the value at [key] if present, otherwise the value interpolated between the
     * nearest surrounding entries. Returns null only if the table is empty.
     */
    fun get(key: K): V? {
        map[key]?.let { return it }

        val ceilingEntry = map.ceilingEntry(key)
        val floorEntry = map.floorEntry(key)

        if (ceilingEntry == null && floorEntry == null) return null
        if (ceilingEntry == null) return floorEntry!!.value
        if (floorEntry == null) return ceilingEntry.value

        val t = keyInterpolator.inverseInterpolate(floorEntry.key, ceilingEntry.key, key)
        return valueInterpolator.interpolate(floorEntry.value, ceilingEntry.value, t)
    }
}
