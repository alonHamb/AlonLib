package org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.Waypoint
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.types.WaypointType

/**
 * The ordinary pure-pursuit waypoint: the robot curves through it without stopping. Most other
 * waypoint types build on this one.
 *
 * SolversLib's Java version has ~5 overloaded constructors covering every combination of
 * (translation+rotation | pose | bare x/y) and (with | without a preferred angle) -- collapsed
 * here into named/default parameters, this port's usual Kotlin-idiom substitute for Java
 * overload sprawl.
 *
 * @param copyMode If true, this waypoint's speed/radius/timeout/angle are overwritten from the
 * previous waypoint in the path when the path initializes (via [inherit]) -- lets you specify a
 * waypoint's position now and its motion settings later, in one shared place.
 */
open class GeneralWaypoint(
    override val pose: Pose2d,
    movementSpeed: Double = 0.0,
    turnSpeed: Double = 0.0,
    followRadius: Double = 0.0,
    preferredAngleRadians: Double? = null,
    internal val copyMode: Boolean = false,
) : Waypoint {

    constructor(
        x: Double,
        y: Double,
        movementSpeed: Double = 0.0,
        turnSpeed: Double = 0.0,
        followRadius: Double = 0.0,
        preferredAngleRadians: Double? = null,
    ) : this(Pose2d(x, y, Rotation2d.fromRadians(preferredAngleRadians ?: 0.0)), movementSpeed, turnSpeed, followRadius, preferredAngleRadians)

    var movementSpeed = normalizeSpeed(movementSpeed)
        private set

    var turnSpeed = normalizeSpeed(turnSpeed)
        private set

    var followRadius = followRadius
        private set

    final override var timeoutMilliseconds: Long = -1
        private set

    private var preferredAngle: Double? = preferredAngleRadians

    val usingPreferredAngle get() = preferredAngle != null

    /** This waypoint's preferred angle, in radians. Throws if [usingPreferredAngle] is false. */
    val preferredAngleRadians: Double get() = preferredAngle ?: error("This waypoint is not using a preferredAngle")

    override val type get() = WaypointType.GENERAL
    override val followDistance get() = followRadius

    fun setMovementSpeed(speed: Double) = apply { movementSpeed = speed }
    fun setTurnSpeed(speed: Double) = apply { turnSpeed = speed }
    fun setFollowRadius(radius: Double) = apply { followRadius = radius }
    fun setPreferredAngle(angleRadians: Double) = apply { preferredAngle = angleRadians }
    fun setTimeout(millis: Long) = apply { timeoutMilliseconds = millis }
    fun disablePreferredAngle() = apply { preferredAngle = null }

    /** Called once when the path initializes; subclasses override to reset per-run state. */
    open fun reset() {}

    /** Copies [waypoint]'s motion settings onto this one, if [copyMode] is enabled. Called by [org.firstinspires.ftc.teamcode.alonlib.purepursuit.Path.init]. */
    fun inherit(waypoint: Waypoint) {
        if (!copyMode) return
        require(waypoint is GeneralWaypoint) { "A $type waypoint cannot inherit the configuration of a ${waypoint.type} waypoint." }
        movementSpeed = waypoint.movementSpeed
        turnSpeed = waypoint.turnSpeed
        followRadius = waypoint.followRadius
        timeoutMilliseconds = waypoint.timeoutMilliseconds
        preferredAngle = if (waypoint.usingPreferredAngle) waypoint.preferredAngleRadians else null
    }

    protected fun normalizeSpeed(raw: Double) = raw.coerceIn(0.0, 1.0)

    override fun toString() = "GeneralWaypoint(${pose.x}, ${pose.y})"
}
