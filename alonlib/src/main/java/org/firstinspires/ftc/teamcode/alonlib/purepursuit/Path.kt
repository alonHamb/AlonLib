package org.firstinspires.ftc.teamcode.alonlib.purepursuit

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.actions.TriggeredAction
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.types.PathType
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.types.WaypointType
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.EndWaypoint
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.GeneralWaypoint
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.InterruptWaypoint
import org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.PointTurnWaypoint
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max

/**
 * A pure-pursuit path: an ordered list of [Waypoint]s, plus everything needed to turn "the
 * robot's current position" into motor powers that follow them. Call [init] once before the
 * first [loop] call; call [loop] every robot loop with the robot's current pose to get the next
 * `[strafe, forward, turn]` motor powers.
 */
class Path(waypoints: List<Waypoint> = emptyList()) : ArrayList<Waypoint>(waypoints) {

    constructor(vararg waypoints: Waypoint) : this(waypoints.toList())

    private var pathType = PathType.WAYPOINT_ORDERING_CONTROLLED
    private var motionProfile: PathMotionProfile = defaultMotionProfile ?: defaultProfile()

    private var timeoutMilliseconds = -1L
    private var startTimeMillis = -1L
    private var lastWaypoint: Waypoint? = null
    private var lastWaypointTimeStampMillis = 0L
    var timedOut = false
        private set

    private var retraceEnabled = true
    private var initComplete = false

    private var retracing = false
    private var retraceMovementSpeed = 1.0
    private var retraceTurnSpeed = 1.0
    private var lastKnownIntersection: Translation2d? = null

    private val triggeredActions = mutableListOf<TriggeredAction>()
    private val interruptActionQueue = ArrayDeque<InterruptWaypoint>()

    /**
     * Initiates the path -- must be called before [loop]. A legal path needs at least 2
     * waypoints, must start with a [org.firstinspires.ftc.teamcode.alonlib.purepursuit.waypoints.StartWaypoint],
     * must end with an [EndWaypoint], and must not contain either type anywhere else.
     */
    fun init() {
        verifyLegality()
        reset()
        for (i in 1 until size) (this[i] as GeneralWaypoint).inherit(this[i - 1])
        initComplete = true
    }

    /**
     * Drives the path to completion using [drive] (strafe, forward, turn powers each loop),
     * [stop] once finished, [pose] to read the robot's current position, and [updateOdometry]
     * called once per loop after driving. Runs [init] itself. Blocks until the path finishes,
     * times out, or loses the path with retrace disabled (returning false in the latter two cases).
     */
    fun followPath(drive: (Double, Double, Double) -> Unit, stop: () -> Unit, pose: () -> Pose2d, updateOdometry: () -> Unit): Boolean {
        init()
        while (!isFinished()) {
            val robotPosition = pose()
            val motorPowers = loop(robotPosition.x, robotPosition.y, robotPosition.rotation.radians)
            drive(motorPowers[0], motorPowers[1], motorPowers[2])
            if (!isFinished() && motorPowers.all { it == 0.0 }) return false
            updateOdometry()
        }
        stop()
        return true
    }

    /**
     * The principal path method: given the robot's current [vPosition]/[hPosition]/[rotation],
     * returns the `[strafe, forward, turn]` motor powers to follow the path. All-zero powers mean
     * the path timed out, lost the path (with retrace disabled), or reached the destination --
     * check [isFinished]/[timedOut] to tell which.
     */
    fun loop(vPosition: Double, hPosition: Double, rotation: Double): DoubleArray {
        check(initComplete) { "You must call init() before calling loop()" }
        if (timedOut) return doubleArrayOf(0.0, 0.0, 0.0)

        if (timeoutMilliseconds != -1L) {
            if (startTimeMillis == -1L) {
                startTimeMillis = System.currentTimeMillis()
            } else if (startTimeMillis + timeoutMilliseconds < System.currentTimeMillis()) {
                timedOut = true
                return doubleArrayOf(0.0, 0.0, 0.0)
            }
        }

        triggeredActions.forEach { it.loop() }
        while (interruptActionQueue.isNotEmpty()) interruptActionQueue.removeFirst().performAction()

        val intersections = mutableListOf<TaggedIntersection>()
        for (i in 1 until size) {
            val linePoint1 = this[i - 1].pose.translation
            val linePoint2 = this[i].pose.translation
            val radius = this[i].followDistance
            val robotPosition = Translation2d(vPosition, hPosition)
            for (point in PurePursuitUtil.lineCircleIntersection(robotPosition, radius, linePoint1, linePoint2)) {
                intersections.add(TaggedIntersection(point, this[i], i))
            }
            val waypointI = this[i]
            if (waypointI is PointTurnWaypoint) {
                val dx = linePoint2.x - vPosition
                val dy = linePoint2.y - hPosition
                val adjustedRadius = hypot(dx, dy) - 1e-9
                if (adjustedRadius < radius) intersections.add(TaggedIntersection(waypointI.pose.translation, waypointI, i))
            }
        }

        if (intersections.isEmpty()) {
            if (retracing) return retrace(vPosition, hPosition, rotation)
            if (retraceEnabled) {
                if (lastKnownIntersection == null) lastKnownIntersection = this[0].pose.translation
                retracing = true
                return retrace(vPosition, hPosition, rotation)
            }
            return doubleArrayOf(0.0, 0.0, 0.0)
        } else {
            retracing = false
        }

        var bestIntersection = intersections[0]
        bestIntersection = when (pathType) {
            PathType.HEADING_CONTROLLED -> selectHeadingControlledIntersection(intersections, Pose2d(vPosition, hPosition, Rotation2d.fromRadians(rotation)))
            PathType.WAYPOINT_ORDERING_CONTROLLED -> selectWaypointOrderingControlledIntersection(intersections)
        }

        if (retraceEnabled) lastKnownIntersection = bestIntersection.intersection

        if (bestIntersection.taggedPoint !== lastWaypoint) {
            lastWaypoint = bestIntersection.taggedPoint
            lastWaypointTimeStampMillis = System.currentTimeMillis()
        }
        if (bestIntersection.taggedPoint.timeoutMilliseconds != -1L) {
            if (System.currentTimeMillis() > lastWaypointTimeStampMillis + bestIntersection.taggedPoint.timeoutMilliseconds) {
                timedOut = true
                return doubleArrayOf(0.0, 0.0, 0.0)
            }
        }

        val robotPos = Pose2d(vPosition, hPosition, Rotation2d.fromRadians(rotation))
        val motorPowers = when (bestIntersection.taggedPoint.type) {
            WaypointType.GENERAL -> handleGeneralIntersection(bestIntersection, robotPos)
            WaypointType.POINT_TURN -> handlePointTurnIntersection(bestIntersection, robotPos)
            WaypointType.INTERRUPT -> handleInterruptIntersection(bestIntersection, robotPos)
            WaypointType.END -> handleInterruptIntersection(bestIntersection, robotPos)
            WaypointType.START -> error("Path has lost integrity.")
        }

        adjustSpeedsWithProfile(motorPowers, bestIntersection, robotPos.translation)
        normalizeMotorSpeeds(motorPowers)
        return motorPowers
    }

    private fun retrace(vPosition: Double, hPosition: Double, rotation: Double): DoubleArray {
        val target = lastKnownIntersection!!
        val motorPowers = PurePursuitUtil.moveToPosition(vPosition, hPosition, rotation, target.x, target.y, rotation, false)
        motorPowers[0] *= retraceMovementSpeed
        motorPowers[1] *= retraceMovementSpeed
        motorPowers[2] *= retraceTurnSpeed
        return motorPowers
    }

    private fun selectHeadingControlledIntersection(intersections: List<TaggedIntersection>, robotPos: Pose2d): TaggedIntersection {
        var best = intersections[0]
        var pointTurnPriority = false
        for (intersection in intersections) {
            val tagged = intersection.taggedPoint
            if (tagged is PointTurnWaypoint) {
                if (!tagged.hasTraversed) {
                    pointTurnPriority = true
                    val bestTagged = best.taggedPoint
                    if (bestTagged !is PointTurnWaypoint) {
                        best = intersection
                    } else if (best.waypointIndex < intersection.waypointIndex) {
                        best = intersection
                    } else if (best.waypointIndex == intersection.waypointIndex) {
                        if (PurePursuitUtil.isInFront(this[intersection.waypointIndex - 1].pose.translation, tagged.pose.translation, intersection.intersection, best.intersection)) {
                            best = intersection
                        }
                    }
                }
            } else if (pointTurnPriority) {
                continue
            } else {
                val relativeAngleToIntersection = atan2(intersection.intersection.y, intersection.intersection.x) - robotPos.rotation.radians
                val relativeAngleToBest = atan2(best.intersection.y, best.intersection.x) - robotPos.rotation.radians
                if (relativeAngleToIntersection < relativeAngleToBest) best = intersection
            }
        }
        return best
    }

    private fun selectWaypointOrderingControlledIntersection(intersections: List<TaggedIntersection>): TaggedIntersection {
        var best = intersections[0]
        var pointTurnPriority = false
        for (intersection in intersections) {
            val tagged = intersection.taggedPoint
            if (tagged is PointTurnWaypoint) {
                if (!tagged.hasTraversed) {
                    pointTurnPriority = true
                    val bestTagged = best.taggedPoint
                    if (bestTagged !is PointTurnWaypoint) {
                        best = intersection
                    } else if (bestTagged.hasTraversed) {
                        best = intersection
                    } else if (best.waypointIndex > intersection.waypointIndex || tagged.hasTraversed) {
                        best = intersection
                    } else if (best.waypointIndex == intersection.waypointIndex) {
                        if (PurePursuitUtil.isInFront(this[intersection.waypointIndex - 1].pose.translation, tagged.pose.translation, intersection.intersection, best.intersection)) {
                            best = intersection
                        }
                    }
                }
            } else if (pointTurnPriority) {
                continue
            } else if (best.waypointIndex < intersection.waypointIndex) {
                best = intersection
            } else if (best.waypointIndex == intersection.waypointIndex) {
                if (PurePursuitUtil.isInFront(this[intersection.waypointIndex - 1].pose.translation, tagged.pose.translation, intersection.intersection, best.intersection)) {
                    best = intersection
                }
            }
        }
        return best
    }

    private fun handleGeneralIntersection(intersection: TaggedIntersection, robotPos: Pose2d): DoubleArray {
        val waypoint = intersection.taggedPoint as GeneralWaypoint
        val cx = robotPos.translation.x
        val cy = robotPos.translation.y
        val ca = robotPos.rotation.radians
        val tx = intersection.intersection.x
        val ty = intersection.intersection.y
        val ta = if (waypoint.usingPreferredAngle) waypoint.preferredAngleRadians else atan2(ty - cy, tx - cx)
        return PurePursuitUtil.moveToPosition(cx, cy, ca, tx, ty, ta, false)
    }

    private fun handlePointTurnIntersection(intersection: TaggedIntersection, robotPos: Pose2d): DoubleArray {
        val waypoint = intersection.taggedPoint as PointTurnWaypoint
        val cx = robotPos.translation.x
        val cy = robotPos.translation.y
        val ca = robotPos.rotation.radians
        val tx = intersection.intersection.x
        val ty = intersection.intersection.y

        return if (!waypoint.hasTraversed && PurePursuitUtil.positionEqualsWithBuffer(robotPos.translation, waypoint.pose.translation, waypoint.positionBuffer)) {
            val next = this[intersection.waypointIndex + 1] as GeneralWaypoint
            val ta: Double
            if (next.usingPreferredAngle) {
                if (PurePursuitUtil.rotationEqualsWithBuffer(ca, next.preferredAngleRadians, waypoint.rotationBuffer)) waypoint.setTraversed()
                ta = next.preferredAngleRadians
            } else {
                val tempTx = next.pose.translation.x
                val tempTy = next.pose.translation.y
                ta = atan2(tempTy - cy, tempTx - cx)
                if (PurePursuitUtil.rotationEqualsWithBuffer(ca, ta, waypoint.rotationBuffer)) waypoint.setTraversed()
            }
            PurePursuitUtil.moveToPosition(cx, cy, ca, tx, ty, ta, true)
        } else {
            val ta = if (waypoint.usingPreferredAngle) waypoint.preferredAngleRadians else atan2(ty - cy, tx - cx)
            PurePursuitUtil.moveToPosition(cx, cy, ca, tx, ty, ta, false)
        }
    }

    private fun handleInterruptIntersection(intersection: TaggedIntersection, robotPos: Pose2d): DoubleArray {
        val waypoint = intersection.taggedPoint as InterruptWaypoint
        val cx = robotPos.translation.x
        val cy = robotPos.translation.y
        val ca = robotPos.rotation.radians
        val tx = intersection.intersection.x
        val ty = intersection.intersection.y

        return if (!waypoint.hasTraversed && PurePursuitUtil.positionEqualsWithBuffer(robotPos.translation, waypoint.pose.translation, waypoint.positionBuffer)) {
            val ta: Double
            if (waypoint.type == WaypointType.END) {
                if (waypoint.usingPreferredAngle && !PurePursuitUtil.rotationEqualsWithBuffer(ca, waypoint.preferredAngleRadians, waypoint.rotationBuffer)) {
                    ta = waypoint.preferredAngleRadians
                } else {
                    (waypoint as EndWaypoint).setTraversed()
                    return doubleArrayOf(0.0, 0.0, 0.0)
                }
            } else {
                val next = this[intersection.waypointIndex + 1] as GeneralWaypoint
                if (next.usingPreferredAngle) {
                    if (PurePursuitUtil.rotationEqualsWithBuffer(ca, next.preferredAngleRadians, waypoint.rotationBuffer)) {
                        waypoint.setTraversed()
                        interruptActionQueue.addLast(waypoint)
                        return doubleArrayOf(0.0, 0.0, 0.0)
                    }
                    ta = next.preferredAngleRadians
                } else {
                    val tempTx = next.pose.translation.x
                    val tempTy = next.pose.translation.y
                    val calculatedTa = atan2(tempTy - cy, tempTx - cx)
                    if (PurePursuitUtil.rotationEqualsWithBuffer(ca, calculatedTa, waypoint.rotationBuffer)) {
                        waypoint.setTraversed()
                        interruptActionQueue.addLast(waypoint)
                        return doubleArrayOf(0.0, 0.0, 0.0)
                    }
                    ta = calculatedTa
                }
            }
            PurePursuitUtil.moveToPosition(cx, cy, ca, tx, ty, ta, true)
        } else {
            val ta = if (waypoint.usingPreferredAngle) waypoint.preferredAngleRadians else atan2(ty - cy, tx - cx)
            PurePursuitUtil.moveToPosition(cx, cy, ca, tx, ty, ta, false)
        }
    }

    /** Sets an overall timeout for the path; if it doesn't finish within [milliseconds], it aborts. */
    fun setPathTimeout(milliseconds: Long) = apply { timeoutMilliseconds = milliseconds }

    /** Not recommended unless you know what you're doing -- default is [PathType.WAYPOINT_ORDERING_CONTROLLED]. */
    fun setPathType(type: PathType) = apply { pathType = type }

    fun setMotionProfile(profile: PathMotionProfile) = apply { motionProfile = profile }

    /** Sets the first `timeouts.size` waypoints' individual timeouts. */
    fun setWaypointTimeouts(vararg timeouts: Long) = apply {
        for (i in 0 until minOf(size, timeouts.size)) (this[i] as? GeneralWaypoint)?.setTimeout(timeouts[i])
    }

    /** Sets every waypoint's individual timeout to the same value. Not recommended. */
    fun setWaypointTimeouts(timeout: Long) = apply {
        forEach { (it as? GeneralWaypoint)?.setTimeout(timeout) }
    }

    /** Configures how fast the robot retraces its path after losing it (both default to 1). */
    fun setRetraceSettings(movementSpeed: Double, turnSpeed: Double) = apply {
        retraceMovementSpeed = movementSpeed.coerceIn(0.0, 1.0)
        retraceTurnSpeed = turnSpeed.coerceIn(0.0, 1.0)
    }

    fun resetTimeouts() = apply {
        timedOut = false
        lastWaypointTimeStampMillis = System.currentTimeMillis()
    }

    /** If the robot loses the path, retrace its moves to try to re-find it. Enabled by default. */
    fun enableRetrace() = apply { retraceEnabled = true }
    fun disableRetrace() = apply { retraceEnabled = false }

    fun addTriggeredActions(vararg actions: TriggeredAction) = apply { triggeredActions.addAll(actions) }
    fun removeTriggeredAction(action: TriggeredAction) = apply { triggeredActions.remove(action) }
    fun clearTriggeredActions() = apply { triggeredActions.clear() }

    /** True if this path has at least 2 waypoints, starts with a start waypoint, ends with an end waypoint, and has neither anywhere else. */
    fun isLegalPath(): Boolean = try {
        verifyLegality()
        initComplete = false
        true
    } catch (e: IllegalStateException) {
        false
    }

    fun isFinished(): Boolean = (lastOrNull() as? EndWaypoint)?.isFinished ?: false

    /** Resets every waypoint/timeout/action. Called by [init]. */
    fun reset() {
        resetTimeouts()
        forEach { (it as? GeneralWaypoint)?.reset() }
        triggeredActions.forEach { it.reset() }
    }

    private fun verifyLegality() {
        check(size >= 2) { "A path must have at least two waypoints." }
        check(this[0].type == WaypointType.START) { "A path must start with a StartWaypoint." }
        check(this[size - 1].type == WaypointType.END) { "A path must end with an EndWaypoint." }
        for (i in 1 until size - 1) {
            check(this[i].type != WaypointType.END && this[i].type != WaypointType.START) {
                "A path must not have end and start waypoints anywhere other than the first and last spot."
            }
        }
    }

    private fun adjustSpeedsWithProfile(speeds: DoubleArray, intersection: TaggedIntersection, robotPos: Translation2d) {
        var awayPoint: Translation2d? = null
        for (i in intersection.waypointIndex - 1 downTo 0) {
            if (this[i].type == WaypointType.START || this[i] is PointTurnWaypoint) {
                awayPoint = this[i].pose.translation
                break
            }
        }
        checkNotNull(awayPoint) { "Path has lost integrity." }

        val toPoint = intersection.taggedPoint.pose.translation
        val ad = hypot(robotPos.x - awayPoint.x, robotPos.y - awayPoint.y)
        val td = hypot(toPoint.x - robotPos.x, toPoint.y - robotPos.y)
        val taggedGeneral = intersection.taggedPoint as GeneralWaypoint

        if (ad < td) {
            motionProfile.processAccelerate(speeds, ad, taggedGeneral.movementSpeed, taggedGeneral.turnSpeed)
        } else {
            motionProfile.processDecelerate(speeds, td, taggedGeneral.movementSpeed, taggedGeneral.turnSpeed)
        }
    }

    private fun normalizeMotorSpeeds(speeds: DoubleArray) {
        val max = max(abs(speeds[0]), abs(speeds[1]))
        if (max > 1) {
            speeds[0] /= max
            speeds[1] /= max
        }
        speeds[2] = speeds[2].coerceIn(-1.0, 1.0)
    }

    private class TaggedIntersection(val intersection: Translation2d, val taggedPoint: Waypoint, val waypointIndex: Int)

    companion object {
        private var defaultMotionProfile: PathMotionProfile? = null

        fun setDefaultMotionProfile(profile: PathMotionProfile) {
            defaultMotionProfile = profile
        }

        /** A simple trapezoid(ish) default profile: full configured speed until within 0.15 units of the target, then a linear ramp. */
        private fun defaultProfile() = object : PathMotionProfile() {
            override fun decelerate(motorSpeeds: DoubleArray, distanceToTarget: Double, speed: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double) {
                scale(motorSpeeds, distanceToTarget, configuredMovementSpeed, configuredTurnSpeed)
            }

            override fun accelerate(motorSpeeds: DoubleArray, distanceFromTarget: Double, speed: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double) {
                scale(motorSpeeds, distanceFromTarget, configuredMovementSpeed, configuredTurnSpeed)
            }

            private fun scale(motorSpeeds: DoubleArray, distance: Double, configuredMovementSpeed: Double, configuredTurnSpeed: Double) {
                if (distance < 0.15) {
                    motorSpeeds[0] *= configuredMovementSpeed * (distance * 10 + 0.1)
                    motorSpeeds[1] *= configuredMovementSpeed * (distance * 10 + 0.1)
                    motorSpeeds[2] *= configuredTurnSpeed
                } else {
                    motorSpeeds[0] *= configuredMovementSpeed
                    motorSpeeds[1] *= configuredMovementSpeed
                    motorSpeeds[2] *= configuredTurnSpeed
                }
            }
        }
    }
}
