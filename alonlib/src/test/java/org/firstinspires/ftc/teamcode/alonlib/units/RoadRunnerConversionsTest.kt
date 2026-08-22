package org.firstinspires.ftc.teamcode.alonlib.units

import com.seattlesolvers.solverslib.geometry.Pose2d as SolversPose2d
import com.seattlesolvers.solverslib.geometry.Rotation2d as SolversRotation2d
import org.junit.Assert.assertEquals
import org.junit.Test

class RoadRunnerConversionsTest {

    private val delta = 1e-9

    @Test
    fun `Rotation2d round-trips between SolversLib and RoadRunner`() {
        val original = SolversRotation2d(1.2)

        val roadRunner = original.toRoadRunner()
        assertEquals(original.radians, roadRunner.log(), delta)

        val back = roadRunner.toSolversLib()
        assertEquals(original.radians, back.radians, delta)
    }

    @Test
    fun `Pose2d round-trips between SolversLib and RoadRunner`() {
        val original = SolversPose2d(3.0, 4.0, SolversRotation2d(0.5))

        val roadRunner = original.toRoadRunner()
        assertEquals(original.x, roadRunner.position.x, delta)
        assertEquals(original.y, roadRunner.position.y, delta)
        assertEquals(original.rotation.radians, roadRunner.heading.log(), delta)

        val back = roadRunner.toSolversLib()
        assertEquals(original.x, back.x, delta)
        assertEquals(original.y, back.y, delta)
        assertEquals(original.rotation.radians, back.rotation.radians, delta)
    }
}
