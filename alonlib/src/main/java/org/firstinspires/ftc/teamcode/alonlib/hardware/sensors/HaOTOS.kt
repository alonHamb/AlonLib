package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/** The SparkFun Optical Tracking Odometry Sensor: absolute position/velocity/acceleration tracking, with an onboard IMU for heading. */
class HaOTOS(val otos: SparkFunOTOS) : com.qualcomm.robotcore.hardware.HardwareDevice by otos {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(SparkFunOTOS::class.java, id))

    fun calibrateImu(numSamples: Int = 255, waitUntilDone: Boolean = true) = otos.calibrateImu(numSamples, waitUntilDone)
    val imuCalibrationProgress get() = otos.imuCalibrationProgress

    var linearUnit: DistanceUnit
        get() = otos.linearUnit
        set(value) { otos.linearUnit = value }

    var angularUnit: AngleUnit
        get() = otos.angularUnit
        set(value) { otos.angularUnit = value }

    var linearScalar: Double
        get() = otos.linearScalar
        set(value) { otos.setLinearScalar(value) }

    var angularScalar: Double
        get() = otos.angularScalar
        set(value) { otos.setAngularScalar(value) }

    fun resetTracking() = otos.resetTracking()

    val status: SparkFunOTOS.Status get() = otos.status

    var offset: SparkFunOTOS.Pose2D
        get() = otos.offset
        set(value) { otos.offset = value }

    var position: SparkFunOTOS.Pose2D
        get() = otos.position
        set(value) { otos.position = value }

    val velocity: SparkFunOTOS.Pose2D get() = otos.velocity
    val acceleration: SparkFunOTOS.Pose2D get() = otos.acceleration
}
