package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import kotlin.math.sqrt

/**
 * Inverse kinematics from a desired [ChassisSpeeds] to four mecanum wheel speeds -- like
 * [MecanumDriveKinematics], but its matching forward-kinematics direction
 * ([toChassisSpeeds]) reads from dead-wheel [OdoWheelSpeeds] (left/right/center) instead of the
 * four drive wheels themselves, for robots whose odometry pods are separate from their drive wheels.
 */
class MecanumOdoKinematics(
    private val frontLeftWheel: Translation2d,
    private val frontRightWheel: Translation2d,
    private val rearLeftWheel: Translation2d,
    private val rearRightWheel: Translation2d,
    private val auxDistance: Double,
    wheelbaseWidth: Double,
) {
    private var inverseKinematics = buildInverseKinematics(frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel)
    private val wheelbaseRadius = wheelbaseWidth / 2.0

    private var prevCenterOfRotation = Translation2d.kZero

    fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds, centerOfRotation: Translation2d = Translation2d.kZero): MecanumDriveWheelSpeeds {
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

    /** Forward kinematics from the dead-wheel [OdoWheelSpeeds] to a chassis velocity. */
    fun toChassisSpeeds(wheelSpeeds: OdoWheelSpeeds): ChassisSpeeds {
        val omega = (wheelSpeeds.right - wheelSpeeds.left) / (wheelbaseRadius * 2.0)
        return ChassisSpeeds(
            (wheelSpeeds.left + wheelSpeeds.right) / 2.0,
            wheelSpeeds.center - auxDistance * omega,
            omega,
        )
    }

    companion object {
        private fun buildInverseKinematics(fl: Translation2d, fr: Translation2d, rl: Translation2d, rr: Translation2d): Matrix {
            val m = Matrix(4, 3)
            m.setRow(0, 1.0, -1.0, -(fl.x + fl.y))
            m.setRow(1, 1.0, 1.0, fr.x - fr.y)
            m.setRow(2, 1.0, 1.0, rl.x - rl.y)
            m.setRow(3, 1.0, -1.0, -(rr.x + rr.y))
            return m * (1.0 / sqrt(2.0))
        }
    }
}
