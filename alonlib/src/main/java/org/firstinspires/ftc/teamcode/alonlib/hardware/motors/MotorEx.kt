package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

/** A [Motor] backed by a [DcMotorEx], adding velocity-unit helpers, current monitoring, and power-write caching. */
open class MotorEx(motorEx: DcMotorEx, gobildaType: GoBILDA = GoBILDA.NONE) : Motor(motorEx, gobildaType) {

    constructor(hardwareMap: HardwareMap, id: String, gobildaType: GoBILDA = GoBILDA.NONE) :
            this(hardwareMap.get(DcMotorEx::class.java, id), gobildaType)

    /** For a motor whose spec isn't one of [GoBILDA]'s presets -- clones the SDK's motor config with a custom [cpr]/[rpm]. */
    constructor(hardwareMap: HardwareMap, id: String, cpr: Double, rpm: Double) : this(hardwareMap.get(DcMotorEx::class.java, id), GoBILDA.NONE) {
        val configType = motorEx.motorType.clone()
        configType.setMaxRPM(rpm)
        configType.setTicksPerRev(cpr)
        motorEx.motorType = configType
        achievableMaxTicksPerSecond = cpr * rpm / 60
    }

    val motorEx: DcMotorEx = motorEx

    /** The minimum power delta (or exactly zero) before [set] actually writes to the motor. */
    var cachingTolerance = 0.0001

    override fun set(output: Double) {
        val power = when (runMode) {
            RunMode.VELOCITY_CONTROL -> {
                val speed = bufferFraction * output * achievableMaxTicksPerSecond
                (veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed)) / achievableMaxTicksPerSecond
            }
            RunMode.POSITION_CONTROL -> output * positionController.calculate(encoder.distance)
            RunMode.RAW_POWER -> output
        }
        writePower(power)
    }

    /** Sets the motor's [RunMode.VELOCITY_CONTROL] target, in ticks/second. */
    fun setVelocity(velocity: Double) = set(velocity / achievableMaxTicksPerSecond)

    /** As [setVelocity], but in an angular rate ([angleUnit]) rather than raw ticks/second. */
    fun setVelocity(velocity: Double, angleUnit: AngleUnit) = setVelocity(cpr * AngleUnit.RADIANS.fromUnit(angleUnit, velocity) / (2 * Math.PI))

    override fun getVelocity() = motorEx.velocity

    fun getAcceleration() = encoder.getAcceleration()

    override fun getDeviceType() = "Extended ${super.getDeviceType()}"

    private fun writePower(power: Double) {
        if (abs(power - lastPower) > cachingTolerance || (power == 0.0 && lastPower != 0.0)) {
            lastPower = power
            motorEx.power = power
        }
    }

    fun getCurrent(currentUnit: CurrentUnit): Double = motorEx.getCurrent(currentUnit)
    fun getCurrentAlert(currentUnit: CurrentUnit): Double = motorEx.getCurrentAlert(currentUnit)
    fun setCurrentAlert(current: Double, unit: CurrentUnit) = apply { motorEx.setCurrentAlert(current, unit) }
    fun isOverCurrent(): Boolean = motorEx.isOverCurrent
}
