package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.teamcode.alonlib.units.millimeters

object DriveConstants {
    val PINPOINT_ODOMETRY_PODS = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD

    val X_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD

    val Y_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD

    val PINPOINT_X_OFFSET = (-84).millimeters

    val PINPOINT_Y_OFFSET = (-168).millimeters

    object LocalizerConstants

    enum class Telemetry {
        Limelight,
        PinPoint
    }
}