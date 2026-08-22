package org.firstinspires.ftc.teamcode.alonlib.math.system

import org.firstinspires.ftc.teamcode.alonlib.robotPrintError

/** Factory functions building [LinearSystem] plant models for common mechanisms. */
object Models {

    /** States: `[position, velocity]`. Inputs: `[voltage]`. Outputs: `[position, velocity]`. */
    fun elevatorSystem(motor: DCMotor, massKg: Double, radiusMeters: Double, gearing: Double): LinearSystem {
        if (massKg <= 0.0) robotPrintError("elevatorSystem: massKg must be greater than zero")
        if (radiusMeters <= 0.0) robotPrintError("elevatorSystem: radiusMeters must be greater than zero")
        if (gearing <= 0.0) robotPrintError("elevatorSystem: gearing must be greater than zero")

        val a = Matrix.fill(
            2, 2,
            0.0, 1.0,
            0.0, -(gearing * gearing) * motor.ktNMPerAmp /
                    (motor.rOhms * radiusMeters * radiusMeters * massKg * motor.kvRadPerSecPerVolt),
        )
        val b = Matrix.vector(0.0, gearing * motor.ktNMPerAmp / (motor.rOhms * radiusMeters * massKg))
        return LinearSystem(a, b, Matrix.eye(2), Matrix(2, 1))
    }

    /** States: `[angular velocity]`. Inputs: `[voltage]`. Outputs: `[angular velocity]`. */
    fun flywheelSystem(motor: DCMotor, jKgMetersSquared: Double, gearing: Double): LinearSystem {
        if (jKgMetersSquared <= 0.0) robotPrintError("flywheelSystem: jKgMetersSquared must be greater than zero")
        if (gearing <= 0.0) robotPrintError("flywheelSystem: gearing must be greater than zero")

        val a = Matrix.vector(-gearing * gearing * motor.ktNMPerAmp / (motor.kvRadPerSecPerVolt * motor.rOhms * jKgMetersSquared))
        val b = Matrix.vector(gearing * motor.ktNMPerAmp / (motor.rOhms * jKgMetersSquared))
        return LinearSystem(a, b, Matrix.eye(1), Matrix(1, 1))
    }

    /** States: `[angular position, angular velocity]`. Inputs: `[voltage]`. Outputs: same as states. */
    fun dcMotorSystem(motor: DCMotor, jKgMetersSquared: Double, gearing: Double): LinearSystem {
        if (jKgMetersSquared <= 0.0) robotPrintError("dcMotorSystem: jKgMetersSquared must be greater than zero")
        if (gearing <= 0.0) robotPrintError("dcMotorSystem: gearing must be greater than zero")

        val a = Matrix.fill(
            2, 2,
            0.0, 1.0,
            0.0, -gearing * gearing * motor.ktNMPerAmp / (motor.kvRadPerSecPerVolt * motor.rOhms * jKgMetersSquared),
        )
        val b = Matrix.vector(0.0, gearing * motor.ktNMPerAmp / (motor.rOhms * jKgMetersSquared))
        return LinearSystem(a, b, Matrix.eye(2), Matrix(2, 1))
    }

    /** States/outputs: `[position, velocity]`. Inputs: `[voltage]`. From SysId-characterized kV/kA. */
    fun dcMotorSystem(kV: Double, kA: Double): LinearSystem {
        if (kV < 0.0) robotPrintError("dcMotorSystem: kV must be greater than or equal to zero")
        if (kA <= 0.0) robotPrintError("dcMotorSystem: kA must be greater than zero")

        val a = Matrix.fill(2, 2, 0.0, 1.0, 0.0, -kV / kA)
        val b = Matrix.vector(0.0, 1.0 / kA)
        return LinearSystem(a, b, Matrix.eye(2), Matrix(2, 1))
    }

    /** States/outputs: `[left velocity, right velocity]`. Inputs: `[left voltage, right voltage]`. */
    fun drivetrainVelocitySystem(
        motor: DCMotor,
        massKg: Double,
        rMeters: Double,
        rbMeters: Double,
        jKgMetersSquared: Double,
        gearing: Double,
    ): LinearSystem {
        if (massKg <= 0.0) robotPrintError("drivetrainVelocitySystem: massKg must be greater than zero")
        if (rMeters <= 0.0) robotPrintError("drivetrainVelocitySystem: rMeters must be greater than zero")
        if (rbMeters <= 0.0) robotPrintError("drivetrainVelocitySystem: rbMeters must be greater than zero")
        if (jKgMetersSquared <= 0.0) robotPrintError("drivetrainVelocitySystem: jKgMetersSquared must be greater than zero")
        if (gearing <= 0.0) robotPrintError("drivetrainVelocitySystem: gearing must be greater than zero")

        val c1 = -(gearing * gearing) * motor.ktNMPerAmp / (motor.kvRadPerSecPerVolt * motor.rOhms * rMeters * rMeters)
        val c2 = gearing * motor.ktNMPerAmp / (motor.rOhms * rMeters)
        val c3 = 1 / massKg + rbMeters * rbMeters / jKgMetersSquared
        val c4 = 1 / massKg - rbMeters * rbMeters / jKgMetersSquared

        val a = Matrix.fill(2, 2, c3 * c1, c4 * c1, c4 * c1, c3 * c1)
        val b = Matrix.fill(2, 2, c3 * c2, c4 * c2, c4 * c2, c3 * c2)
        return LinearSystem(a, b, Matrix.eye(2), Matrix(2, 2))
    }

    /** States: `[angle, angular velocity]`. Inputs: `[voltage]`. Outputs: same as states. */
    fun singleJointedArmSystem(motor: DCMotor, jKgMetersSquared: Double, gearing: Double): LinearSystem {
        if (jKgMetersSquared <= 0.0) robotPrintError("singleJointedArmSystem: jKgMetersSquared must be greater than zero")
        if (gearing <= 0.0) robotPrintError("singleJointedArmSystem: gearing must be greater than zero")

        val a = Matrix.fill(
            2, 2,
            0.0, 1.0,
            0.0, -(gearing * gearing) * motor.ktNMPerAmp / (motor.kvRadPerSecPerVolt * motor.rOhms * jKgMetersSquared),
        )
        val b = Matrix.vector(0.0, gearing * motor.ktNMPerAmp / (motor.rOhms * jKgMetersSquared))
        return LinearSystem(a, b, Matrix.eye(2), Matrix(2, 1))
    }

    /** States/outputs: `[velocity]`. Inputs: `[voltage]`. From SysId-characterized kV/kA. */
    fun identifyVelocitySystem(kV: Double, kA: Double): LinearSystem {
        if (kV < 0.0) robotPrintError("identifyVelocitySystem: kV must be greater than or equal to zero")
        if (kA <= 0.0) robotPrintError("identifyVelocitySystem: kA must be greater than zero")

        return LinearSystem(Matrix.vector(-kV / kA), Matrix.vector(1.0 / kA), Matrix.vector(1.0), Matrix.vector(0.0))
    }

    /** States/outputs: `[position, velocity]`. Inputs: `[voltage]`. From SysId-characterized kV/kA. */
    fun identifyPositionSystem(kV: Double, kA: Double) = dcMotorSystem(kV, kA)

    /** States/outputs: `[left velocity, right velocity]`. Inputs: `[left voltage, right voltage]`. From SysId gains. */
    fun identifyDrivetrainSystem(kVLinear: Double, kALinear: Double, kVAngular: Double, kAAngular: Double): LinearSystem {
        if (kVLinear <= 0.0) robotPrintError("identifyDrivetrainSystem: kVLinear must be greater than zero")
        if (kALinear <= 0.0) robotPrintError("identifyDrivetrainSystem: kALinear must be greater than zero")
        if (kVAngular <= 0.0) robotPrintError("identifyDrivetrainSystem: kVAngular must be greater than zero")
        if (kAAngular <= 0.0) robotPrintError("identifyDrivetrainSystem: kAAngular must be greater than zero")

        val a1 = 0.5 * -(kVLinear / kALinear + kVAngular / kAAngular)
        val a2 = 0.5 * -(kVLinear / kALinear - kVAngular / kAAngular)
        val b1 = 0.5 * (1.0 / kALinear + 1.0 / kAAngular)
        val b2 = 0.5 * (1.0 / kALinear - 1.0 / kAAngular)

        val a = Matrix.fill(2, 2, a1, a2, a2, a1)
        val b = Matrix.fill(2, 2, b1, b2, b2, b1)
        return LinearSystem(a, b, Matrix.eye(2), Matrix(2, 2))
    }

    /** Same as the 4-arg overload, but taking angular gains in volts per radian/sec(²) and converting via [trackwidthMeters]. */
    fun identifyDrivetrainSystem(
        kVLinear: Double,
        kALinear: Double,
        kVAngular: Double,
        kAAngular: Double,
        trackwidthMeters: Double,
    ): LinearSystem {
        if (trackwidthMeters <= 0.0) robotPrintError("identifyDrivetrainSystem: trackwidthMeters must be greater than zero")
        return identifyDrivetrainSystem(kVLinear, kALinear, kVAngular * 2.0 / trackwidthMeters, kAAngular * 2.0 / trackwidthMeters)
    }
}
