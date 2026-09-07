package org.firstinspires.ftc.teamcode.alonlib.math.control

import org.firstinspires.ftc.teamcode.alonlib.math.system.LinearSystem
import org.firstinspires.ftc.teamcode.alonlib.math.system.Matrix
import org.firstinspires.ftc.teamcode.alonlib.math.system.Models

/** Computes per-side feedforward voltages for a differential drivetrain, from its SysId-characterized kV/kA gains. */
class DifferentialDriveFeedforward {

    val kVLinear: Double
    val kALinear: Double
    val kVAngular: Double
    val kAAngular: Double

    private val plant: LinearSystem

    /** [kVAngular]/[kAAngular] here are per (radians/sec), converted internally via [trackwidthMeters]. */
    constructor(kVLinear: Double, kALinear: Double, kVAngular: Double, kAAngular: Double, trackwidthMeters: Double) :
            this(kVLinear, kALinear, kVAngular * 2.0 / trackwidthMeters, kAAngular * 2.0 / trackwidthMeters)

    /** [kVAngular]/[kAAngular] here are already per (meters/sec), matching the drivetrain's linear units. */
    constructor(kVLinear: Double, kALinear: Double, kVAngular: Double, kAAngular: Double) {
        this.kVLinear = kVLinear
        this.kALinear = kALinear
        this.kVAngular = kVAngular
        this.kAAngular = kAAngular
        plant = Models.identifyDrivetrainSystem(kVLinear, kALinear, kVAngular, kAAngular)
    }

    /** The feedforward voltages to go from the current per-side velocities to the next ones over [dtSeconds]. */
    fun calculate(
        currentLeftVelocity: Double,
        nextLeftVelocity: Double,
        currentRightVelocity: Double,
        nextRightVelocity: Double,
        dtSeconds: Double,
    ): DifferentialDriveWheelVoltages {
        val feedforward = LinearPlantInversionFeedforward(plant, dtSeconds)
        val r = Matrix.vector(currentLeftVelocity, currentRightVelocity)
        val nextR = Matrix.vector(nextLeftVelocity, nextRightVelocity)
        val u = feedforward.calculate(r, nextR)
        return DifferentialDriveWheelVoltages(u[0, 0], u[1, 0])
    }
}
