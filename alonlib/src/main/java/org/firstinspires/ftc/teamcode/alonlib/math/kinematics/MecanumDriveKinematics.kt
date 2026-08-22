package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix

/**
 * Converts between [ChassisSpeeds] and four-wheel [MecanumDriveWheelSpeeds]/[MecanumDriveWheelPositions],
 * given each wheel's location relative to the robot's center.
 *
 * Inverse kinematics (chassis speed -> wheel speeds) is a straightforward matrix multiply by the
 * wheel locations. Forward kinematics (wheel speeds -> chassis speed) is overdetermined -- 4
 * equations, 3 unknowns -- so it uses the Moore-Penrose [Matrix.pseudoInverse] for a least-squares
 * solution.
 */
class MecanumDriveKinematics(
    val frontLeftWheel: Translation2d,
    val frontRightWheel: Translation2d,
    val rearLeftWheel: Translation2d,
    val rearRightWheel: Translation2d,
) : Kinematics<MecanumDriveWheelSpeeds, MecanumDriveWheelPositions> {

    private var inverseKinematics = buildInverseKinematics(frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel)
    private val forwardKinematics = inverseKinematics.pseudoInverse()

    private var prevCenterOfRotation = Translation2d.kZero

    /**
     * Inverse kinematics with a variable [centerOfRotation] -- e.g. set it to a corner of the
     * robot to pivot around that corner instead of the physical center.
     */
    fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds, centerOfRotation: Translation2d): MecanumDriveWheelSpeeds {
        if (centerOfRotation != prevCenterOfRotation) {
            inverseKinematics = buildInverseKinematics(
                frontLeftWheel - centerOfRotation,
                frontRightWheel - centerOfRotation,
                rearLeftWheel - centerOfRotation,
                rearRightWheel - centerOfRotation,
            )
            prevCenterOfRotation = centerOfRotation
        }

        val chassisSpeedsVector = Matrix.vector(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega)
        val wheelsVector = inverseKinematics * chassisSpeedsVector
        return MecanumDriveWheelSpeeds(wheelsVector[0, 0], wheelsVector[1, 0], wheelsVector[2, 0], wheelsVector[3, 0])
    }

    override fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds) = toWheelSpeeds(chassisSpeeds, Translation2d.kZero)

    override fun toChassisSpeeds(wheelSpeeds: MecanumDriveWheelSpeeds): ChassisSpeeds {
        val wheelSpeedsVector = Matrix.vector(wheelSpeeds.frontLeft, wheelSpeeds.frontRight, wheelSpeeds.rearLeft, wheelSpeeds.rearRight)
        val chassisSpeedsVector = forwardKinematics * wheelSpeedsVector
        return ChassisSpeeds(chassisSpeedsVector[0, 0], chassisSpeedsVector[1, 0], chassisSpeedsVector[2, 0])
    }

    override fun toTwist2d(start: MecanumDriveWheelPositions, end: MecanumDriveWheelPositions) = toTwist2d(
        MecanumDriveWheelPositions(
            end.frontLeft - start.frontLeft,
            end.frontRight - start.frontRight,
            end.rearLeft - start.rearLeft,
            end.rearRight - start.rearRight,
        )
    )

    /** Forward kinematics from per-wheel distance deltas directly, for odometry. */
    fun toTwist2d(wheelDeltas: MecanumDriveWheelPositions): Twist2d {
        val wheelDeltasVector = Matrix.vector(wheelDeltas.frontLeft, wheelDeltas.frontRight, wheelDeltas.rearLeft, wheelDeltas.rearRight)
        val twist = forwardKinematics * wheelDeltasVector
        return Twist2d(twist[0, 0], twist[1, 0], twist[2, 0])
    }

    override fun interpolate(startValue: MecanumDriveWheelPositions, endValue: MecanumDriveWheelPositions, t: Double) =
        startValue.interpolate(endValue, t)

    companion object {
        private fun buildInverseKinematics(fl: Translation2d, fr: Translation2d, rl: Translation2d, rr: Translation2d): Matrix {
            val m = Matrix(4, 3)
            m.setRow(0, 1.0, -1.0, -(fl.x + fl.y))
            m.setRow(1, 1.0, 1.0, fr.x - fr.y)
            m.setRow(2, 1.0, 1.0, rl.x - rl.y)
            m.setRow(3, 1.0, -1.0, -(rr.x + rr.y))
            return m
        }
    }
}
