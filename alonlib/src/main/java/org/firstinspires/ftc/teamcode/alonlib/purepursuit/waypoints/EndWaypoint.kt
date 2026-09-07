package org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.actions.InterruptAction
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.types.WaypointType

/** The final waypoint of every [org.firstinspires.ftc.teamcode.alonlib.purepursuit.Path] -- an [InterruptWaypoint] whose action marks the path finished, and can't be changed. */
class EndWaypoint(
    pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    positionBuffer: Double,
    rotationBuffer: Double,
    preferredAngleRadians: Double? = null,
) : InterruptWaypoint(pose, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer, preferredAngleRadians) {

    var isFinished = false
        private set

    init {
        super.setAction(InterruptAction { isFinished = true })
    }

    override fun setTraversed() {
        isFinished = true
    }

    override fun setAction(newAction: InterruptAction): InterruptWaypoint =
        throw IllegalArgumentException("You cannot change the action of an end waypoint.")

    override fun reset() {
        super.reset()
        isFinished = false
    }

    override val type get() = WaypointType.END

    override fun toString() = "EndWaypoint(${pose.x}, ${pose.y})"
}
