package org.firstinspires.ftc.teamcode.alonlib.hardware

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d

/** Common surface for gyro/IMU wrappers -- see [RevIMU] and (preferred on modern hubs) [org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaIMU]. */
abstract class GyroEx : HardwareDevice {
    abstract fun init()
    abstract fun getHeading(): Double
    abstract fun getAbsoluteHeading(): Double
    abstract fun getAngles(): DoubleArray
    abstract fun getRotation2d(): Rotation2d
    abstract fun reset()
}
