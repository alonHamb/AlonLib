package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d

/** Wraps the FTC SDK's universal [IMU] interface -- works with any modern hub-mounted IMU, regardless of vendor. */
class HaIMU(val imu: IMU) : com.qualcomm.robotcore.hardware.HardwareDevice by imu {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(IMU::class.java, id))

    // --- setup ---

    fun initialize(parameters: IMU.Parameters) = imu.initialize(parameters)
    fun resetYaw() = imu.resetYaw()

    // --- readings ---

    /** The current yaw/pitch/roll. */
    val yawPitchRollAngles get() = imu.robotYawPitchRollAngles

    /** Yaw as a [Rotation2d], the form the rest of AlonLib's geometry/odometry expects. */
    val rotation2d get() = Rotation2d.fromDegrees(imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))

    fun getOrientation(axesReference: AxesReference, axesOrder: AxesOrder, angleUnit: AngleUnit) =
        imu.getRobotOrientation(axesReference, axesOrder, angleUnit)

    val orientationAsQuaternion get() = imu.robotOrientationAsQuaternion

    fun getAngularVelocity(angleUnit: AngleUnit) = imu.getRobotAngularVelocity(angleUnit)
}
