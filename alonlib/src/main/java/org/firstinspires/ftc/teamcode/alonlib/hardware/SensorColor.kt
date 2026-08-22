package org.firstinspires.ftc.teamcode.alonlib.hardware

import android.graphics.Color
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.HardwareMap

/** A basic ARGB [ColorSensor] wrapper. */
open class SensorColor(private val colorSensor: ColorSensor) : HardwareDevice {

    constructor(hardwareMap: HardwareMap, name: String) : this(hardwareMap.get(ColorSensor::class.java, name))

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

    override fun disable() = colorSensor.close()
    override fun getDeviceType() = "Color Sensor"
}
