package org.firstinspires.ftc.teamcode.alonlib.math.geometry

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

/**
 * A free vector in 2D Cartesian space -- as opposed to [Translation2d], which is a *point*.
 * Kept distinct from [Translation2d] (matching SolversLib's own split) because drivebase code
 * ([org.firstinspires.ftc.teamcode.alonlib.drives]) wants vector algebra (dot/project/normalize)
 * that doesn't make sense for a point.
 */
class Vector2d(val x: Double = 0.0, val y: Double = 0.0) {

    constructor(pose: Pose2d) : this(pose.x, pose.y)

    /** Rotates the vector counter-clockwise by [angleRadians]. */
    fun rotateBy(angleRadians: Double): Vector2d {
        val cosA = kotlin.math.cos(angleRadians)
        val sinA = kotlin.math.sin(angleRadians)
        return Vector2d(x * cosA - y * sinA, x * sinA + y * cosA)
    }


    fun angle() = atan2(y, x)

    operator fun plus(other: Vector2d) = Vector2d(x + other.x, y + other.y)
    operator fun minus(other: Vector2d) = this + (-other)
    operator fun unaryMinus() = Vector2d(-x, -y)
    operator fun times(scalar: Double) = Vector2d(x * scalar, y * scalar)
    operator fun div(scalar: Double) = Vector2d(x / scalar, y / scalar)

    fun dot(other: Vector2d) = x * other.x + y * other.y
    fun magnitude() = hypot(x, y)
    fun scalarProject(other: Vector2d) = dot(other) / other.magnitude()
    fun scale(scalar: Double) = this * scalar
    fun normalize() = scale(1.0 / magnitude())
    fun project(other: Vector2d) = other * (dot(other) / other.dot(other))

    override fun equals(other: Any?): Boolean {
        if (other !is Vector2d) return false
        return abs(x - other.x) < 1e-9 && abs(y - other.y) < 1e-9
    }

    override fun hashCode() = 31 * x.hashCode() + y.hashCode()

    override fun toString() = "( $x, $y )"
}
