package org.firstinspires.ftc.teamcode.alonlib.math.kinematics

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Twist2d
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

/**
 * Converts between [ChassisSpeeds] and per-module [SwerveModuleState]/[SwerveModulePosition] for
 * an arbitrary number (2+) of swerve modules at arbitrary locations, via the same
 * pseudoinverse-based least-squares approach as [MecanumDriveKinematics].
 *
 * Module order is whatever order [moduleTranslations] were passed in -- every array this class
 * takes or returns (module states, positions, headings) must use that same order.
 */
class SwerveDriveKinematics(vararg val moduleTranslations: Translation2d) :
        Kinematics<Array<SwerveModuleState>, Array<SwerveModulePosition>> {

    init {
        if (moduleTranslations.size < 2) {
            robotPrintError("a swerve drive requires at least two modules")
        }
    }

    private val numModules = moduleTranslations.size
    private var moduleHeadings = Array(numModules) { Rotation2d.kZero }
    private var inverseKinematics = buildInverseKinematics(moduleTranslations, Translation2d.kZero)
    private val forwardKinematics = inverseKinematics.pseudoInverse()

    private var prevCenterOfRotation = Translation2d.kZero

    /** Resets the module headings used when commanding a zero chassis speed (modules hold their last angle). */
    fun resetHeadings(vararg headings: Rotation2d) {
        if (headings.size != numModules) {
            robotPrintError("resetHeadings got ${headings.size} headings for $numModules modules")
            return
        }
        moduleHeadings = Array(numModules) { headings[it] }
    }

    /**
     * Inverse kinematics with a variable [centerOfRotation]. When [chassisSpeeds] is exactly zero,
     * every module keeps its last-commanded heading instead of snapping to zero degrees.
     */
    fun toSwerveModuleStates(chassisSpeeds: ChassisSpeeds, centerOfRotation: Translation2d = Translation2d.kZero): Array<SwerveModuleState> {
        if (chassisSpeeds.vx == 0.0 && chassisSpeeds.vy == 0.0 && chassisSpeeds.omega == 0.0) {
            return Array(numModules) { SwerveModuleState(0.0, moduleHeadings[it]) }
        }

        if (centerOfRotation != prevCenterOfRotation) {
            inverseKinematics = buildInverseKinematics(moduleTranslations, centerOfRotation)
            prevCenterOfRotation = centerOfRotation
        }

        val chassisSpeedsVector = Matrix.vector(chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega)
        val moduleStatesMatrix = inverseKinematics * chassisSpeedsVector

        return Array(numModules) { i ->
            val x = moduleStatesMatrix[i * 2, 0]
            val y = moduleStatesMatrix[i * 2 + 1, 0]

            val speed = hypot(x, y)
            val angle = if (speed > 1e-6) Rotation2d(x, y) else moduleHeadings[i]

            moduleHeadings[i] = angle
            SwerveModuleState(speed, angle)
        }
    }

    override fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds) = toSwerveModuleStates(chassisSpeeds)

    override fun toChassisSpeeds(wheelSpeeds: Array<SwerveModuleState>): ChassisSpeeds {
        if (wheelSpeeds.size != numModules) {
            robotPrintError("toChassisSpeeds got ${wheelSpeeds.size} module states for $numModules modules")
            return ChassisSpeeds()
        }

        val moduleStatesMatrix = Matrix(numModules * 2, 1)
        for (i in 0 until numModules) {
            val module = wheelSpeeds[i]
            moduleStatesMatrix[i * 2, 0] = module.speedMetersPerSecond * module.angle.cos
            moduleStatesMatrix[i * 2 + 1, 0] = module.speedMetersPerSecond * module.angle.sin
        }

        val chassisSpeedsVector = forwardKinematics * moduleStatesMatrix
        return ChassisSpeeds(chassisSpeedsVector[0, 0], chassisSpeedsVector[1, 0], chassisSpeedsVector[2, 0])
    }

    /** Forward kinematics from per-module distance deltas directly, for odometry. */
    fun toTwist2d(vararg moduleDeltas: SwerveModulePosition): Twist2d {
        if (moduleDeltas.size != numModules) {
            robotPrintError("toTwist2d got ${moduleDeltas.size} module deltas for $numModules modules")
            return Twist2d()
        }

        val moduleDeltaMatrix = Matrix(numModules * 2, 1)
        for (i in 0 until numModules) {
            val module = moduleDeltas[i]
            moduleDeltaMatrix[i * 2, 0] = module.distanceMeters * module.angle.cos
            moduleDeltaMatrix[i * 2 + 1, 0] = module.distanceMeters * module.angle.sin
        }

        val chassisDeltaVector = forwardKinematics * moduleDeltaMatrix
        return Twist2d(chassisDeltaVector[0, 0], chassisDeltaVector[1, 0], chassisDeltaVector[2, 0])
    }

    override fun toTwist2d(start: Array<SwerveModulePosition>, end: Array<SwerveModulePosition>): Twist2d {
        if (start.size != end.size) {
            robotPrintError("inconsistent number of modules")
            return Twist2d()
        }
        return toTwist2d(*Array(start.size) { SwerveModulePosition(end[it].distanceMeters - start[it].distanceMeters, end[it].angle) })
    }

    override fun interpolate(startValue: Array<SwerveModulePosition>, endValue: Array<SwerveModulePosition>, t: Double): Array<SwerveModulePosition> {
        if (startValue.size != endValue.size) {
            robotPrintError("inconsistent number of modules")
            return startValue
        }
        return Array(startValue.size) { startValue[it].interpolate(endValue[it], t) }
    }

    companion object {
        /** Scales every module's speed down (preserving ratios) if any exceeds [attainableMaxSpeed], mutating [moduleStates] in place. */
        fun desaturateWheelSpeeds(moduleStates: Array<SwerveModuleState>, attainableMaxSpeed: Double) {
            val realMaxSpeed = moduleStates.maxOf { abs(it.speedMetersPerSecond) }
            if (realMaxSpeed > attainableMaxSpeed) {
                for (module in moduleStates) {
                    module.speedMetersPerSecond = module.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeed
                }
            }
        }

        /**
         * Like the two-arg [desaturateWheelSpeeds], but also backs off translation/rotation together
         * (not just per-module) so the commanded chassis motion's shape is preserved -- avoids
         * joystick-edge saturation distorting the direction of travel. Mutates [moduleStates] in place.
         */
        fun desaturateWheelSpeeds(
            moduleStates: Array<SwerveModuleState>,
            desiredChassisSpeed: ChassisSpeeds,
            attainableMaxModuleSpeed: Double,
            attainableMaxTranslationalSpeed: Double,
            attainableMaxRotationalVelocity: Double,
        ) {
            val realMaxSpeed = moduleStates.maxOf { abs(it.speedMetersPerSecond) }

            if (attainableMaxTranslationalSpeed == 0.0 || attainableMaxRotationalVelocity == 0.0 || realMaxSpeed == 0.0) {
                return
            }

            val translationalK = hypot(desiredChassisSpeed.vx, desiredChassisSpeed.vy) / attainableMaxTranslationalSpeed
            val rotationalK = abs(desiredChassisSpeed.omega) / attainableMaxRotationalVelocity
            val k = max(translationalK, rotationalK)
            val scale = min(k * attainableMaxModuleSpeed / realMaxSpeed, 1.0)
            for (module in moduleStates) {
                module.speedMetersPerSecond *= scale
            }
        }

        private fun buildInverseKinematics(modules: Array<out Translation2d>, centerOfRotation: Translation2d): Matrix {
            val m = Matrix(modules.size * 2, 3)
            for (i in modules.indices) {
                val module = modules[i]
                m.setRow(i * 2, 1.0, 0.0, -module.y + centerOfRotation.y)
                m.setRow(i * 2 + 1, 0.0, 1.0, module.x - centerOfRotation.x)
            }
            return m
        }
    }
}
