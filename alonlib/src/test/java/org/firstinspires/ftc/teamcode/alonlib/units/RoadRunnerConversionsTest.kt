package org.firstinspires.ftc.teamcode.alonlib.units

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Pose2d
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.junit.Assert.assertEquals
import org.junit.Test

class RoadRunnerConversionsTest {

    private val delta = 1e-9

    @Test
    fun `Rotation2d round-trips between AlonLib and RoadRunner`() {
        val original = Rotation2d.fromRadians(1.2)

        val roadRunner = original.toRoadRunner()
        assertEquals(original.radians, roadRunner.log(), delta)

        val back = roadRunner.toRotation2d()
        assertEquals(original.radians, back.radians, delta)
    }

    @Test
    fun `Pose2d round-trips between AlonLib and RoadRunner`() {
        val original = Pose2d(3.0, 4.0, Rotation2d.fromRadians(0.5))

        val roadRunner = original.toRoadRunner()
        assertEquals(original.x, roadRunner.position.x, delta)
        assertEquals(original.y, roadRunner.position.y, delta)
        assertEquals(original.rotation.radians, roadRunner.heading.log(), delta)

        val back = roadRunner.toPose2d()
        assertEquals(original.x, back.x, delta)
        assertEquals(original.y, back.y, delta)
        assertEquals(original.rotation.radians, back.rotation.radians, delta)
    }
}
