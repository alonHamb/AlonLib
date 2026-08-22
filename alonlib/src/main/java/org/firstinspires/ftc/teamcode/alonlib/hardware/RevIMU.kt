package org.firstinspires.ftc.teamcode.alonlib.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d

/**
 * The REV Expansion/Control Hub's built-in [BNO055IMU]. Prefer [org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaIMU]
 * on modern hubs (it wraps the universal `IMU` interface, working with any vendor); this exists
 * for the legacy BNO055-specific API some older configs still expect.
 */
open class RevIMU(private val revIMU: BNO055IMU) : GyroEx() {

    constructor(hardwareMap: HardwareMap, imuName: String = "imu") : this(hardwareMap.get(BNO055IMU::class.java, imuName))

    private var globalHeadingOffset = 0.0
    private var multiplier = 1

    override fun init() = init(
        BNO055IMU.Parameters().apply {
            angleUnit = BNO055IMU.AngleUnit.DEGREES
            calibrationDataFile = "BNO055IMUCalibration.json"
            loggingEnabled = true
            loggingTag = "IMU"
        },
    )

    fun init(parameters: BNO055IMU.Parameters) {
        revIMU.initialize(parameters)
        globalHeadingOffset = 0.0
    }

    /** Flips the sign of every heading this reports. */
    fun invertGyro() {
        multiplier *= -1
    }

    override fun getHeading() = getAbsoluteHeading() - globalHeadingOffset

    override fun getAbsoluteHeading() = revIMU.angularOrientation.firstAngle.toDouble() * multiplier

    override fun getAngles(): DoubleArray {
        val orientation = revIMU.angularOrientation
        return doubleArrayOf(orientation.firstAngle.toDouble(), orientation.secondAngle.toDouble(), orientation.thirdAngle.toDouble())
    }

    override fun getRotation2d(): Rotation2d = Rotation2d.fromDegrees(getHeading())

    override fun disable() = revIMU.close()

    override fun reset() {
        globalHeadingOffset += getHeading()
    }

    override fun getDeviceType() = "Rev Expansion Hub IMU"

    fun getRevIMU() = revIMU
}
