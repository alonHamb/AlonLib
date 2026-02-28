package org.firstinspires.ftc.teamcode.subsystems.vision

import com.seattlesolvers.solverslib.geometry.Pose2d
import com.seattlesolvers.solverslib.geometry.Translation2d
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.meters

object VisionConstants {
    val LIMELIGHT_ACCURACY_RANGE = 0.6.meters..3.5.meters
    val FAR_SHOOTING_RANGE = 3.meters..5.meters

    val BLUE_GOAL_TAG_ID = 20

    val RED_GOAL_TAG_ID = 24
    val BLUE_GOAL_TARGET = Pose2d(Translation2d(-1.7, -1.7), 54.046000.degrees) // translation is in metres from the center of the field

    val RED_GOAL_TARGET = Pose2d(Translation2d(-1.7, 1.7), 305.954.degrees) // translation is in metres from the center of the field
}