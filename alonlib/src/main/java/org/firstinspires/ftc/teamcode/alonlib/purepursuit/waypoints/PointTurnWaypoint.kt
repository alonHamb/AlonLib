package org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.types.WaypointType

/** A waypoint the robot comes to a complete stop at, turns in place towards the next waypoint, then continues -- rather than curving through it like a plain [GeneralWaypoint]. */
open class PointTurnWaypoint(
    pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    positionBuffer: Double,
    rotationBuffer: Double,
    preferredAngleRadians: Double? = null,
) : GeneralWaypoint(pose, movementSpeed, turnSpeed, followRadius, preferredAngleRadians) {

    var positionBuffer: Double = verifyBuffer(positionBuffer)
        private set

    var rotationBuffer: Double = verifyBuffer(rotationBuffer)
        private set

    var hasTraversed = false
        private set

    fun setPositionBuffer(buffer: Double) = apply { positionBuffer = verifyBuffer(buffer) }
    fun setRotationBuffer(buffer: Double) = apply { rotationBuffer = verifyBuffer(buffer) }

    open fun setTraversed() {
        hasTraversed = true
    }

    override fun reset() {
        hasTraversed = false
    }

    override val type get() = WaypointType.POINT_TURN

    private fun verifyBuffer(buffer: Double): Double {
        require(buffer > 0) { "The buffer must be > 0" }
        return buffer
    }

    override fun toString() = "PointTurnWaypoint(${pose.x}, ${pose.y})"
}
