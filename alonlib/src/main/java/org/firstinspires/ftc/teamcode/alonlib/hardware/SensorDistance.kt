package org.firstinspires.ftc.teamcode.alonlib.hardware

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/** A sensor that reports distance to whatever it's pointed at. */
interface SensorDistance : HardwareDevice {
    fun getDistance(unit: DistanceUnit): Double
}
