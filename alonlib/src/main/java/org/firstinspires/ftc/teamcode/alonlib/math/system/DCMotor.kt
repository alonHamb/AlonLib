package org.firstinspires.ftc.teamcode.alonlib.math.system

/**
 * The physical constants of a DC motor (or a gearbox of identical ones), for use in state-space
 * plant models (see [Models]). All fields are in SI units and already account for [numMotors].
 */
class DCMotor(
    val nominalVoltageVolts: Double,
    stallTorqueNewtonMeters: Double,
    stallCurrentAmps: Double,
    freeCurrentAmps: Double,
    val freeSpeedRadPerSec: Double,
    numMotors: Int = 1,
) {
    val stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors
    val stallCurrentAmps = stallCurrentAmps * numMotors
    val freeCurrentAmps = freeCurrentAmps * numMotors

    /** Motor internal resistance. */
    val rOhms = nominalVoltageVolts / this.stallCurrentAmps

    /** Motor velocity constant. */
    val kvRadPerSecPerVolt = freeSpeedRadPerSec / (nominalVoltageVolts - rOhms * this.freeCurrentAmps)

    /** Motor torque constant. */
    val ktNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps

    /** The current drawn at [speedRadiansPerSec] under [voltageInputVolts]. */
    fun getCurrent(speedRadiansPerSec: Double, voltageInputVolts: Double) =
        -1.0 / kvRadPerSecPerVolt / rOhms * speedRadiansPerSec + 1.0 / rOhms * voltageInputVolts

    /** The current drawn to produce [torqueNm]. */
    fun getCurrent(torqueNm: Double) = torqueNm / ktNMPerAmp

    /** The torque produced by [currentAmps]. */
    fun getTorque(currentAmps: Double) = currentAmps * ktNMPerAmp

    /** The voltage needed to produce [torqueNm] at [speedRadiansPerSec]. */
    fun getVoltage(torqueNm: Double, speedRadiansPerSec: Double) =
        1.0 / kvRadPerSecPerVolt * speedRadiansPerSec + 1.0 / ktNMPerAmp * rOhms * torqueNm

    /** The angular speed produced by [torqueNm] at [voltageInputVolts]. */
    fun getSpeed(torqueNm: Double, voltageInputVolts: Double) =
        voltageInputVolts * kvRadPerSecPerVolt - 1.0 / ktNMPerAmp * torqueNm * rOhms * kvRadPerSecPerVolt

    /** A copy of this motor with [gearboxReduction] applied (output torque multiplied, output speed divided). */
    fun withReduction(gearboxReduction: Double) = DCMotor(
        nominalVoltageVolts,
        stallTorqueNewtonMeters * gearboxReduction,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRadPerSec / gearboxReduction,
        1,
    )

    companion object {
        private fun rpmToRadPerSec(rpm: Double) = rpm * 2.0 * Math.PI / 60.0

        fun cim(numMotors: Int = 1) = DCMotor(12.0, 2.42, 133.0, 2.7, rpmToRadPerSec(5310.0), numMotors)
        fun vex775Pro(numMotors: Int = 1) = DCMotor(12.0, 0.71, 134.0, 0.7, rpmToRadPerSec(18730.0), numMotors)
        fun neo(numMotors: Int = 1) = DCMotor(12.0, 2.6, 105.0, 1.8, rpmToRadPerSec(5676.0), numMotors)
        fun miniCim(numMotors: Int = 1) = DCMotor(12.0, 1.41, 89.0, 3.0, rpmToRadPerSec(5840.0), numMotors)
        fun bag(numMotors: Int = 1) = DCMotor(12.0, 0.43, 53.0, 1.8, rpmToRadPerSec(13180.0), numMotors)
        fun andymarkRs775_125(numMotors: Int = 1) = DCMotor(12.0, 0.28, 18.0, 1.6, rpmToRadPerSec(5800.0), numMotors)
        fun banebotsRs775(numMotors: Int = 1) = DCMotor(12.0, 0.72, 97.0, 2.7, rpmToRadPerSec(13050.0), numMotors)
        fun andymark9015(numMotors: Int = 1) = DCMotor(12.0, 0.36, 71.0, 3.7, rpmToRadPerSec(14270.0), numMotors)
        fun banebotsRs550(numMotors: Int = 1) = DCMotor(12.0, 0.38, 84.0, 0.4, rpmToRadPerSec(19000.0), numMotors)
        fun neo550(numMotors: Int = 1) = DCMotor(12.0, 0.97, 100.0, 1.4, rpmToRadPerSec(11000.0), numMotors)
        fun falcon500(numMotors: Int = 1) = DCMotor(12.0, 4.69, 257.0, 1.5, rpmToRadPerSec(6380.0), numMotors)
        fun falcon500Foc(numMotors: Int = 1) = DCMotor(12.0, 5.84, 304.0, 1.5, rpmToRadPerSec(6080.0), numMotors)
        fun romiBuiltIn(numMotors: Int = 1) = DCMotor(4.5, 0.1765, 1.25, 0.13, rpmToRadPerSec(150.0), numMotors)
        fun krakenX60(numMotors: Int = 1) = DCMotor(12.0, 7.09, 366.0, 2.0, rpmToRadPerSec(6000.0), numMotors)
        fun krakenX60Foc(numMotors: Int = 1) = DCMotor(12.0, 9.37, 483.0, 2.0, rpmToRadPerSec(5800.0), numMotors)
        fun neoVortex(numMotors: Int = 1) = DCMotor(12.0, 3.60, 211.0, 3.6, rpmToRadPerSec(6784.0), numMotors)
    }
}
