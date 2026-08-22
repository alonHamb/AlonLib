package org.firstinspires.ftc.teamcode.alonlib.purepursuit

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/** Standalone math helpers behind [Path]'s pure-pursuit algorithm. */
object PurePursuitUtil {

    /** Wraps [angle] (radians) into `[-pi, pi]`. */
    fun angleWrap(angle: Double): Double =
        if (angle > 0) ((angle + Math.PI) % (Math.PI * 2)) - Math.PI else ((angle - Math.PI) % (Math.PI * 2)) + Math.PI

    /** True if [point1] is farther along the line ([linePoint1] -> [linePoint2]) than [point2]. Both points are assumed to lie on the line. */
    fun isInFront(linePoint1: Translation2d, linePoint2: Translation2d, point1: Translation2d, point2: Translation2d): Boolean {
        if (linePoint1.x < linePoint2.x && point1.x < point2.x) return false
        if (linePoint1.y < linePoint2.y && point1.y < point2.y) return false
        return true
    }

    /** True if [p1] and [p2] are within [buffer] of each other on both axes. */
    fun positionEqualsWithBuffer(p1: Translation2d, p2: Translation2d, buffer: Double): Boolean =
        p1.x - buffer < p2.x && p1.x + buffer > p2.x && p1.y - buffer < p2.y && p1.y + buffer > p2.y

    /** True if angles [a1]/[a2] (radians) are within [buffer] of each other. */
    fun rotationEqualsWithBuffer(a1: Double, a2: Double, buffer: Double): Boolean = a1 - buffer < a2 && a1 + buffer > a2

    /**
     * The raw `[strafe, forward, turn]` motor powers to drive from `(cx, cy, ca)` towards `(tx, ty, ta)`
     * (all in radians for the angles). If [turnOnly], only the turn component is populated.
     */
    fun moveToPosition(cx: Double, cy: Double, ca: Double, tx: Double, ty: Double, ta: Double, turnOnly: Boolean): DoubleArray {
        if (turnOnly) return doubleArrayOf(0.0, 0.0, angleWrap(ca + ta) / Math.PI)

        val dx = tx - cx
        val dy = ty - cy

        val absoluteAngle = atan2(dy, dx)
        val distance = hypot(dx, dy)
        val relativeAngle = angleWrap(absoluteAngle + ca)

        val relativeX = distance * kotlin.math.cos(relativeAngle)
        val relativeY = distance * kotlin.math.sin(relativeAngle)

        val powerX = relativeX / (abs(relativeX) + abs(relativeY))
        val powerY = relativeY / (abs(relativeX) + abs(relativeY))
        val powerTurn = angleWrap(ca + ta) / Math.PI

        return doubleArrayOf(powerX, powerY, powerTurn)
    }

    /** Every point where the line ([linePoint1] -> [linePoint2]) crosses the circle centered at [circleCenter] with [radius], bounded to the line segment. */
    fun lineCircleIntersection(circleCenter: Translation2d, radius: Double, linePoint1: Translation2d, linePoint2: Translation2d): List<Translation2d> {
        // Ported from FTC team 11115 Gluten Free's code (via SolversLib).
        val baX = linePoint2.x - linePoint1.x
        val baY = linePoint2.y - linePoint1.y
        val caX = circleCenter.x - linePoint1.x
        val caY = circleCenter.y - linePoint1.y

        val a = baX * baX + baY * baY
        val bBy2 = baX * caX + baY * caY
        val c = caX * caX + caY * caY - radius * radius

        val pBy2 = bBy2 / a
        val q = c / a

        val disc = pBy2 * pBy2 - q
        if (disc < 0) return emptyList()

        val sqrtDisc = sqrt(disc)
        val scale1 = -pBy2 + sqrtDisc
        val scale2 = -pBy2 - sqrtDisc

        val p1 = Translation2d(linePoint1.x - baX * scale1, linePoint1.y - baY * scale1)
        val allPoints = if (disc == 0.0) listOf(p1) else listOf(p1, Translation2d(linePoint1.x - baX * scale2, linePoint1.y - baY * scale2))

        val maxX = max(linePoint1.x, linePoint2.x)
        val maxY = max(linePoint1.y, linePoint2.y)
        val minX = min(linePoint1.x, linePoint2.x)
        val minY = min(linePoint1.y, linePoint2.y)

        return allPoints.filter { it.x in minX..maxX && it.y in minY..maxY }
    }
}
