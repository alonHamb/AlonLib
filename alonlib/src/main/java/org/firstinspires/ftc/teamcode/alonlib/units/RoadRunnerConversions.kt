package org.firstinspires.ftc.teamcode.alonlib.units

import com.seattlesolvers.solverslib.geometry.Pose2d as SolversPose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d as SolversRotation2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import com.acmerobotics.roadrunner.Pose2d as RoadRunnerPose2d
import com.acmerobotics.roadrunner.Rotation2d as RoadRunnerRotation2d

/**
 * RoadRunner has its own [RoadRunnerPose2d]/[RoadRunnerRotation2d] geometry types, separate from
 * the SolversLib ones used everywhere else in AlonLib (e.g. [HaPinPoint][org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaPinPoint]).
 * These extensions convert between the two at the boundary, so hardware wrappers can stay on
 * SolversLib's types while RoadRunner-specific code (drive/localizer/trajectories) uses its own.
 */

fun SolversRotation2d.toRoadRunner(): RoadRunnerRotation2d = RoadRunnerRotation2d.exp(this.radians)

fun RoadRunnerRotation2d.toSolversLib(): SolversRotation2d = SolversRotation2d(this.log())

fun SolversPose2d.toRoadRunner(): RoadRunnerPose2d = RoadRunnerPose2d(this.x, this.y, this.rotation.radians)

fun RoadRunnerPose2d.toSolversLib(): SolversPose2d =
    SolversPose2d(Translation2d(this.position.x, this.position.y), SolversRotation2d(this.heading.log()))
