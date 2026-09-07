package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap

/** The REV Blinkin LED driver: a servo-PWM-controlled LED strip with a fixed set of built-in patterns. */
class HaBlinkinLedDriver(val blinking: RevBlinkinLedDriver) : com.qualcomm.robotcore.hardware.HardwareDevice by blinking {

	constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(RevBlinkinLedDriver::class.java, id))

	var pattern: RevBlinkinLedDriver.BlinkinPattern? = null
		set(value) {
			field = value
			if (value != null) blinking.setPattern(value)
		}
}
