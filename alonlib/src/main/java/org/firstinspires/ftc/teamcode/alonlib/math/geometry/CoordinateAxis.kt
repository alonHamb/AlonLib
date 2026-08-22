package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import kotlin.math.sqrt

/** A coordinate system axis within the North-West-Up (NWU) coordinate system, for use with [CoordinateSystem]. */
class CoordinateAxis(x: Double, y: Double, z: Double) {

    internal val axis: Translation3d

    init {
        val norm = sqrt(x * x + y * y + z * z)
        axis = Translation3d(x / norm, y / norm, z / norm)
    }

    companion object {
        private val n = CoordinateAxis(1.0, 0.0, 0.0)
        private val s = CoordinateAxis(-1.0, 0.0, 0.0)
        private val e = CoordinateAxis(0.0, -1.0, 0.0)
        private val w = CoordinateAxis(0.0, 1.0, 0.0)
        private val u = CoordinateAxis(0.0, 0.0, 1.0)
        private val d = CoordinateAxis(0.0, 0.0, -1.0)

        /** +X in the NWU coordinate system. */
        fun N() = n

        /** -X in the NWU coordinate system. */
        fun S() = s

        /** -Y in the NWU coordinate system. */
        fun E() = e

        /** +Y in the NWU coordinate system. */
        fun W() = w

        /** +Z in the NWU coordinate system. */
        fun U() = u

        /** -Z in the NWU coordinate system. */
        fun D() = d
    }
}
