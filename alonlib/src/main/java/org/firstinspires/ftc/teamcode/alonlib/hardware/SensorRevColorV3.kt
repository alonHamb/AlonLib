package org.firstinspires.ftc.teamcode.alonlib.hardware

import android.graphics.Color
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/** The REV Color Sensor V3: ARGB plus a built-in distance reading. */
open class SensorRevColorV3(val colorSensor: RevColorSensorV3, private val defaultDistanceUnit: DistanceUnit = DistanceUnit.CM) : HardwareDevice {

    constructor(hardwareMap: HardwareMap, name: String, distanceUnit: DistanceUnit = DistanceUnit.CM) :
            this(hardwareMap.colorSensor.get(name) as RevColorSensorV3, distanceUnit)

    fun hsvToArgb(alpha: Int, hsv: FloatArray): IntArray {
        val color = Color.HSVToColor(alpha, hsv)
        return intArrayOf(Color.alpha(color), Color.red(color), Color.green(color), Color.blue(color))
    }

    fun rgbToHsv(red: Int, green: Int, blue: Int, hsv: FloatArray): FloatArray {
        Color.RGBToHSV(red, green, blue, hsv)
        return hsv
    }

    fun getArgb() = intArrayOf(alpha(), red(), green(), blue())

    fun alpha() = colorSensor.alpha()
    fun red() = colorSensor.red()
    fun green() = colorSensor.green()
    fun blue() = colorSensor.blue()

    fun distance(unit: DistanceUnit = defaultDistanceUnit): Double = colorSensor.getDistance(unit)

    override fun disable() = colorSensor.close()
    override fun getDeviceType() = "REV Color Sensor v3"
}
