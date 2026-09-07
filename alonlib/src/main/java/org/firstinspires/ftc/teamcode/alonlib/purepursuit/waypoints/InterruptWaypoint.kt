package org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.actions.InterruptAction
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.types.WaypointType

/** A [PointTurnWaypoint] that also performs [action] once the robot stops and turns to face it, before continuing -- for "do something mid-path". */
open class InterruptWaypoint(
    pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    positionBuffer: Double,
    rotationBuffer: Double,
    preferredAngleRadians: Double? = null,
    private var action: InterruptAction = InterruptAction {},
) : PointTurnWaypoint(pose, movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer, preferredAngleRadians) {

    var actionPerformed = false
        private set

    open fun setAction(newAction: InterruptAction) = apply { action = newAction }

    /** Runs [action], if it hasn't already fired since the last [reset]. */
    fun performAction() {
        if (!actionPerformed) {
            action.doAction()
            actionPerformed = true
        }
    }

    override fun reset() {
        super.reset()
        actionPerformed = false
    }

    override val type get() = WaypointType.INTERRUPT

    override fun toString() = "InterruptWaypoint(${pose.x}, ${pose.y})"
}
