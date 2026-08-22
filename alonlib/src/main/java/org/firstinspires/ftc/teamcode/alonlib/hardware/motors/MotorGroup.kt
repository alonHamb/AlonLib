package org.firstinspires.ftc.teamcode.alonlib.hardware.motors

/** Links a [leader] motor and its [followers] into one [Motor]: the leader is driven normally, and every follower mirrors its output. */
open class MotorGroup(private val leader: Motor, private vararg val followers: Motor) : Motor(leader.motor), Iterable<Motor> {

    private val group: List<Motor> = listOf(leader, *followers)

    override fun set(output: Double) {
        leader.set(output)
        followers.forEach { it.set(leader.get()) }
    }

    override fun get() = leader.get()
    override fun getRawPower() = leader.getRawPower()

    /** Every motor's last-set target speed, as a fraction of output. */
    val speeds get() = group.map { it.get() }

    override fun getVelocity() = leader.getCorrectedVelocity()

    /** Every motor's current velocity, in units of distance/second (ticks/second by default). */
    val velocities get() = group.map { it.rate }

    override fun iterator() = group.iterator()

    override fun setDistancePerPulse(distancePerPulse: Double): Motor.Encoder {
        followers.forEach { it.setDistancePerPulse(distancePerPulse) }
        return leader.setDistancePerPulse(distancePerPulse)
    }

    /** Every motor's current position, in units of distance (ticks by default). */
    val positions get() = group.map { it.distance }

    override fun setRunMode(mode: RunMode) = apply { leader.setRunMode(mode) }

    override fun setZeroPowerBehavior(behavior: ZeroPowerBehavior) = apply { group.forEach { it.setZeroPowerBehavior(behavior) } }

    override fun resetEncoder() = leader.resetEncoder()
    override fun stopAndResetEncoder() = leader.stopAndResetEncoder()
    override fun setPositionCoefficient(kp: Double) = leader.setPositionCoefficient(kp)
    override fun atTargetPosition() = leader.atTargetPosition()
    override fun setTargetPosition(target: Int) = leader.setTargetPosition(target)
    override fun setTargetDistance(target: Double) = leader.setTargetDistance(target)
    override fun setPositionTolerance(tolerance: Double) = leader.setPositionTolerance(tolerance)
    override fun setVeloCoefficients(kp: Double, ki: Double, kd: Double) = leader.setVeloCoefficients(kp, ki, kd)
    override fun setFeedforwardCoefficients(ks: Double, kv: Double, ka: Double) = leader.setFeedforwardCoefficients(ks, kv, ka)
    override fun getInverted() = leader.getInverted()

    /** Inverts every motor in the group -- this affects each one's own speed sign, not a "mirror" relationship between them. */
    override fun setInverted(isInverted: Boolean) = apply { group.forEach { it.setInverted(isInverted) } }

    override fun disable() = group.forEach { it.disable() }
    override fun getDeviceType() = "Motor Group"
    override fun stopMotor() = group.forEach { it.stopMotor() }
}
