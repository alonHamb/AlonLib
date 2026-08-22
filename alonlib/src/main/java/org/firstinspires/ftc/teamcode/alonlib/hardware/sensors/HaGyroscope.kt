package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.GyroSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/** The legacy [GyroSensor] interface -- superseded by [HaIMU] on modern hubs, but still SDK-supported for older gyro modules. */
class HaGyroscope(val gyroSensor: GyroSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by gyroSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(GyroSensor::class.java, id))

    fun calibrate() = gyroSensor.calibrate()
    val isCalibrating get() = gyroSensor.isCalibrating

    val heading get() = gyroSensor.heading
    val rotationFraction get() = gyroSensor.rotationFraction

    val rawX get() = gyroSensor.rawX()
    val rawY get() = gyroSensor.rawY()
    val rawZ get() = gyroSensor.rawZ()

    fun resetZAxisIntegrator() = gyroSensor.resetZAxisIntegrator()

    /** If this device also implements the SDK's [com.qualcomm.robotcore.hardware.Gyroscope] interface, its angular velocity. */
    fun getAngularVelocity(angleUnit: AngleUnit) = (gyroSensor as com.qualcomm.robotcore.hardware.Gyroscope).getAngularVelocity(angleUnit)
}
