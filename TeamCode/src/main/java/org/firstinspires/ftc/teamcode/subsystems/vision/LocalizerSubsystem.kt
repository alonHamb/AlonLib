package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.robotcore.hardware.GyroSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.RobotMap.Drive.PINPOINT_ID
import org.firstinspires.ftc.teamcode.RobotMap.Vision.LIMELIGHT_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.sensors.HaLimelight3A
import org.firstinspires.ftc.teamcode.alonlib.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.alonlib.units.distanceTo
import org.firstinspires.ftc.teamcode.alonlib.units.horizontalAngleTo
import org.firstinspires.ftc.teamcode.alonlib.units.meters
import org.firstinspires.ftc.teamcode.alonlib.units.radians
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.Telemetry.Limelight
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.Telemetry.PinPoint
import org.firstinspires.ftc.teamcode.subsystems.vision.LocalizerConstants.BLUE_GOAL_TAG_ID
import org.firstinspires.ftc.teamcode.subsystems.vision.LocalizerConstants.BLUE_GOAL_TARGET
import org.firstinspires.ftc.teamcode.subsystems.vision.LocalizerConstants.LIMELIGHT_ACCURACY_RANGE
import org.firstinspires.ftc.teamcode.subsystems.vision.LocalizerConstants.RED_GOAL_TAG_ID
import org.firstinspires.ftc.teamcode.subsystems.vision.LocalizerConstants.RED_GOAL_TARGET

class LocalizerSubsystem(hardwareMap: HardwareMap, var telemetry: Telemetry, val telemetryLevel: TelemetryLevel) : SubsystemBase() {
    // --- hardware declarations ---
    val limelight = HaLimelight3A(hardwareMap, LIMELIGHT_ID)

    val pinPoint = HaPinPoint(hardwareMap, PINPOINT_ID, PINPOINT_ODOMETRY_PODS)

    val imu: GyroSensor = hardwareMap.gyroSensor.get("imu")
    // --- state getters and setters ---

    var currentLocalizer = PinPoint

    val latestBotPose2d: Pose2d
        get() {
            when (isInLimelightAccuracyRange) {
                true  -> {
                    currentLocalizer = Limelight
                    pinPoint.position = limelight.latestPose2d
                    return limelight.latestPose2d
                }

                false -> {
                    currentLocalizer = PinPoint
                    return pinPoint.position
                }
            }
        }
    val pinPointDistanceToRedGoal get() = pinPoint.position.distanceTo(RED_GOAL_TARGET).meters
    val pinPointDistanceToBlueGoal get() = pinPoint.position.distanceTo(BLUE_GOAL_TARGET).meters
    val latestBotPose2D
        get() = Pose2D(
            DistanceUnit.METER,
            latestBotPose2d.x,
            latestBotPose2d.y,
            AngleUnit.DEGREES,
            latestBotPose2d.heading
                      )
    val isInRedGoalRange get() = distanceToRedTarget in LIMELIGHT_ACCURACY_RANGE
    val distanceToRedTarget get() = latestBotPose2d.distanceTo(RED_GOAL_TARGET).meters
    val angleToRedTarget get() = latestBotPose2d.horizontalAngleTo(RED_GOAL_TARGET)
    val isInBlueGoalRange get() = distanceToBlueTarget in LIMELIGHT_ACCURACY_RANGE
    val distanceToBlueTarget get() = latestBotPose2d.distanceTo(BLUE_GOAL_TARGET).meters
    val angleToBlueTarget get() = latestBotPose2d.horizontalAngleTo(BLUE_GOAL_TARGET)
    val isInLimelightAccuracyRange: Boolean
        get() {
            return when (limelight.detectedTags?.get(0)?.fiducialId) {
                BLUE_GOAL_TAG_ID -> pinPointDistanceToBlueGoal in LIMELIGHT_ACCURACY_RANGE
                RED_GOAL_TAG_ID  -> pinPointDistanceToRedGoal in LIMELIGHT_ACCURACY_RANGE
                null             -> false
                else             -> {
                    false
                }
            }
        }


    // --- telemetry ---
    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing     -> {
                telemetry.addLine("--- Vision Subsystem ---")
                telemetry.addData("detected tags", limelight.detectedTags)
                telemetry.addData("is in limelight detection range", isInLimelightAccuracyRange)
                telemetry.addData("current botPose2d", latestBotPose2d)
                telemetry.addData("current botPose2D", latestBotPose2D)
                telemetry.addData("is in red goal range", isInRedGoalRange)
                telemetry.addData("distance to red goal target", distanceToRedTarget)
                telemetry.addData("is in blue goal range", isInBlueGoalRange)
                telemetry.addData("distance to blue goal target", distanceToBlueTarget)
            }
        }
    }

    // --- periodic function ---
    override fun periodic() {
        limelight.UpdateMegaTag2RobotHeading(latestBotPose2d.heading.radians)
        updateTelemetry()
    }
}
