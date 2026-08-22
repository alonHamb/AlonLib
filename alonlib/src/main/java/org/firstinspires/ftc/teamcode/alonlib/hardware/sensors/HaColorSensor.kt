package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/**
 * A normalized color sensor (works with any vendor via the SDK's [NormalizedColorSensor]
 * interface). If the underlying device is also a [DistanceSensor] (e.g. a REV Color Sensor V3),
 * [distance] is available too; otherwise it throws.
 */
class HaColorSensor(val colorSensor: NormalizedColorSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by colorSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(NormalizedColorSensor::class.java, id))

    val normalizedColors get() = colorSensor.normalizedColors

    var gain: Float
        get() = colorSensor.gain
        set(value) { colorSensor.gain = value }

    /** Requires the underlying device to also be a [DistanceSensor] (e.g. a REV Color Sensor V3). */
    fun distance(unit: DistanceUnit): Double = (colorSensor as DistanceSensor).getDistance(unit)
}
