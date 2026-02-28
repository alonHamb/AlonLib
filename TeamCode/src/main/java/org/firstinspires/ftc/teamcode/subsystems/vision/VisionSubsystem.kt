package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.RobotMap.Vision.LIMELIGHT_ID
import org.firstinspires.ftc.teamcode.alonlib.sensors.HaLimelight3A
import org.firstinspires.ftc.teamcode.alonlib.units.distanceTo
import org.firstinspires.ftc.teamcode.alonlib.units.meters
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.BLUE_GOAL_TAG_ID
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.BLUE_GOAL_TARGET
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.DYNAMIC_SHOOTING_RANGE
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.LIMELIGHT_ACCURACY_RANGE
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.RED_GOAL_TAG_ID
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants.RED_GOAL_TARGET

class VisionSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry) : SubsystemBase() {
    // --- hardware declarations ---
    val limelight = HaLimelight3A(hardwareMap, LIMELIGHT_ID)

    // --- state getters and setters ---
    val latestBotPose2d get() = limelight.latestPose2d

    val latestBotPose2D get() = Pose2D(DistanceUnit.METER, limelight.latestPose2d.x, limelight.latestPose2d.y, AngleUnit.DEGREES, limelight.latestPose2d.heading)

    val latestBotPose3d: Pose3D get() = limelight.latestResult.botpose_MT2

    val isWithinRedGoalRange get() = limelight.firstDetectedTag.fiducialId == 24 && (distanceToRedTarget in LIMELIGHT_ACCURACY_RANGE)
    val distanceToRedTarget get() = latestBotPose2d.distanceTo(RED_GOAL_TARGET).meters
    val isWithinBlueGoalRange get() = limelight.firstDetectedTag.fiducialId == 20 && (distanceToBlueTarget in LIMELIGHT_ACCURACY_RANGE)
    val distanceToBlueTarget get() = latestBotPose2d.distanceTo(BLUE_GOAL_TARGET).meters
    val isWithinLimelightAccuracyRange get() = isWithinRedGoalRange || isWithinBlueGoalRange

    val isWithinDynamicShootingRange: Boolean
        get() =
            when (limelight.firstDetectedTagId) {
                BLUE_GOAL_TAG_ID -> distanceToBlueTarget in DYNAMIC_SHOOTING_RANGE
                RED_GOAL_TAG_ID -> distanceToRedTarget in DYNAMIC_SHOOTING_RANGE
                else -> false
            }


    // --- telemetry ---
    fun addTelemetry() {
        telemetry.addLine("--- Vision Subsystem ---")
        telemetry.addData("detected tags", limelight.detectedTags)
        telemetry.addData("current botPose2d", latestBotPose2d)
        telemetry.addData("current botPose2D", latestBotPose2D)
        telemetry.addData("current botPose3d", latestBotPose3d)
        telemetry.addData("is in red goal range", isWithinRedGoalRange)
        telemetry.addData("distance to red goal target", distanceToRedTarget)
        telemetry.addData("is in blue goal range", isWithinRedGoalRange)
        telemetry.addData("distance to blue goal target", distanceToBlueTarget)
        telemetry.addData("is in limelight detection range", isWithinLimelightAccuracyRange)
    }

}
