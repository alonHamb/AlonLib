package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import android.graphics.Color
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.alonlib.units.Length
import org.firstinspires.ftc.teamcode.alonlib.units.meters

/**
 * A normalized color sensor (works with any vendor via the SDK's [NormalizedColorSensor]
 * interface). If the underlying device is also a plain [ColorSensor] (nearly all of them are,
 * e.g. a REV Color Sensor V3), raw ARGB ([getArgb]/[alpha]/[red]/[green]/[blue]) is available too
 * alongside [normalizedColors]; if it's also a [DistanceSensor] (again, e.g. a REV Color Sensor
 * V3), so is [distance]. Both throw if the underlying device doesn't implement that interface.
 */
class HaColorSensor(val colorSensor: NormalizedColorSensor) : com.qualcomm.robotcore.hardware.HardwareDevice by colorSensor {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(NormalizedColorSensor::class.java, id))

    val normalizedColors get() = colorSensor.normalizedColors

    var gain: Float
        get() = colorSensor.gain
        set(value) {
            colorSensor.gain = value
        }

    fun hsvToArgb(alpha: Int, hsv: FloatArray): IntArray {
        val color = Color.HSVToColor(alpha, hsv)
        return intArrayOf(Color.alpha(color), Color.red(color), Color.green(color), Color.blue(color))
    }

    fun rgbToHsv(red: Int, green: Int, blue: Int, hsv: FloatArray): FloatArray {
        Color.RGBToHSV(red, green, blue, hsv)
        return hsv
    }

    fun getArgb() = intArrayOf(alpha, red, green, blue)

    /** Requires the underlying device to also be a plain [ColorSensor] (nearly all of them are). */
    val alpha = (colorSensor as ColorSensor).alpha()
    val red = (colorSensor as ColorSensor).red()
    val green = (colorSensor as ColorSensor).green()
    val blue = (colorSensor as ColorSensor).blue()


    /** ONLY IF TEH SENSOR IS ALSO A DISTANCE SENSOR */
    val distance: Length = (colorSensor as DistanceSensor).getDistance(DistanceUnit.METER).meters

}
