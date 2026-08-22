package org.firstinspires.ftc.teamcode.alonlib.p2p

import org.firstinspires.ftc.teamcode.alonlib.math.angleModulus
import org.firstinspires.ftc.teamcode.alonlib.math.control.PIDController
import org.firstinspires.ftc.teamcode.alonlib.math.filter.SlewRateLimiter
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Transform2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.ChassisSpeeds
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * A simple point-to-point field-centric controller: drives straight towards [target] (magnitude
 * from [translationalController], direction from the straight-line bearing to it) while
 * [headingController] independently turns to face [target]'s rotation.
 *
 * Unlike SolversLib's version, this doesn't take an `AngleUnit` -- [Rotation2d] is already
 * unit-agnostic (always canonically radians internally, with both `.radians`/`.degrees`
 * accessors), so there's nothing left for a separate angle-unit parameter to disambiguate.
 */
class P2PController(
    val translationalController: PIDController,
    val headingController: PIDController,
    start: Pose2d = Pose2d.kZero,
    target: Pose2d = Pose2d.kZero,
    positionTolerance: Double,
    angularToleranceRadians: Double,
) {
    var target: Pose2d = target
        private set

    private var current: Pose2d = start

    var error: Transform2d = Transform2d()
        private set

    private var magnitudeLimiter: SlewRateLimiter? = null
    private var headingLimiter: SlewRateLimiter? = null

    init {
        updateError()
        setTolerance(positionTolerance, angularToleranceRadians)
    }

    /** The field-centric chassis speeds to drive from the robot's current pose [pv] towards [target]. */
    fun calculate(pv: Pose2d): ChassisSpeeds {
        current = pv
        updateError()

        val errorX = target.x - current.x
        val errorY = target.y - current.y

        val distanceToTarget = hypot(errorX, errorY)
        val errorAngle = atan2(errorY, errorX)
        var magnitude = translationalController.calculate(0.0, distanceToTarget)
        magnitudeLimiter?.let { magnitude = it.calculate(magnitude) }

        val xVal = magnitude * cos(errorAngle)
        val yVal = magnitude * sin(errorAngle)

        val headingError = angleModulus(target.rotation.radians - current.rotation.radians)
        var headingVal = headingController.calculate(0.0, headingError)
        headingLimiter?.let { headingVal = it.calculate(headingVal) }

        return ChassisSpeeds(xVal, yVal, headingVal)
    }

    fun setSlewRateLimiters(magnitudeLimiter: SlewRateLimiter?, headingLimiter: SlewRateLimiter?) = apply {
        this.magnitudeLimiter = magnitudeLimiter
        this.headingLimiter = headingLimiter
    }

    fun setTarget(target: Pose2d) {
        this.target = target
    }

    fun setTolerance(positionTolerance: Double, angularToleranceRadians: Double) {
        translationalController.setTolerance(positionTolerance)
        headingController.setTolerance(angularToleranceRadians)
    }

    fun atTarget() = translationalController.atSetPoint() && headingController.atSetPoint()

    private fun updateError() {
        val errorX = target.x - current.x
        val errorY = target.y - current.y
        val errorHeading = angleModulus(target.rotation.radians - current.rotation.radians)
        error = Transform2d(Translation2d(errorX, errorY), Rotation2d.fromRadians(errorHeading))
    }
}
