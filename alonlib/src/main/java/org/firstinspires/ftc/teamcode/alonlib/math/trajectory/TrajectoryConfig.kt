package org.firstinspires.ftc.teamcode.alonlib.math.trajectory

import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.DifferentialDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.kinematics.SwerveDriveKinematics
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint.DifferentialDriveKinematicsConstraint
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint.MecanumDriveKinematicsConstraint
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint.SwerveDriveKinematicsConstraint
import org.firstinspires.ftc.teamcode.alonlib.math.trajectory.constraint.TrajectoryConstraint

/** The start/end velocity, velocity/acceleration caps, custom constraints, and reversed flag used to generate a [Trajectory]. */
class TrajectoryConfig(val maxVelocityMetersPerSecond: Double, val maxAccelerationMetersPerSecondSq: Double) {

    val constraints = mutableListOf<TrajectoryConstraint>()

    var startVelocityMetersPerSecond = 0.0
    var endVelocityMetersPerSecond = 0.0
    var reversed = false

    fun addConstraint(constraint: TrajectoryConstraint) = apply { constraints.add(constraint) }

    fun addConstraints(constraints: List<TrajectoryConstraint>) = apply { this.constraints.addAll(constraints) }

    /** Caps every wheel's speed of a differential drivetrain at [maxVelocityMetersPerSecond]. */
    fun setKinematics(kinematics: DifferentialDriveKinematics) =
        addConstraint(DifferentialDriveKinematicsConstraint(kinematics, maxVelocityMetersPerSecond))

    /** Caps every wheel's speed of a mecanum drivetrain at [maxVelocityMetersPerSecond]. */
    fun setKinematics(kinematics: MecanumDriveKinematics) =
        addConstraint(MecanumDriveKinematicsConstraint(kinematics, maxVelocityMetersPerSecond))

    /** Caps every module's speed of a swerve drivetrain at [maxVelocityMetersPerSecond]. */
    fun setKinematics(kinematics: SwerveDriveKinematics) =
        addConstraint(SwerveDriveKinematicsConstraint(kinematics, maxVelocityMetersPerSecond))
}
