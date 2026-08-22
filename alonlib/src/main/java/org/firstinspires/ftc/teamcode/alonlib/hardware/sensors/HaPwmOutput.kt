package org.firstinspires.ftc.teamcode.alonlib.hardware.sensors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PWMOutput

/** A raw PWM output pin. */
class HaPwmOutput(val pwmOutput: PWMOutput) : com.qualcomm.robotcore.hardware.HardwareDevice by pwmOutput {

    constructor(hardwareMap: HardwareMap, id: String) : this(hardwareMap.get(PWMOutput::class.java, id))

    /** The output pulse width, in microseconds. */
    var pulseWidthOutputTimeMicros: Int
        get() = pwmOutput.pulseWidthOutputTime
        set(value) { pwmOutput.pulseWidthOutputTime = value }

    /** The full PWM period, in microseconds. */
    var pulseWidthPeriodMicros: Int
        get() = pwmOutput.pulseWidthPeriod
        set(value) { pwmOutput.pulseWidthPeriod = value }
}
