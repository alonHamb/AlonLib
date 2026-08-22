package org.firstinspires.ftc.teamcode.alonlib.commands

import org.firstinspires.ftc.teamcode.alonlib.units.milliseconds
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class FactoriesTest {

    @Test
    fun `wait builds a WaitCommand for the given duration`() {
        val command = wait(500.milliseconds)
        assertTrue(command is WaitCommand)
    }

    @Test
    fun `waitUntil builds a WaitUntilCommand wrapping the condition`() {
        val command = waitUntil { true }
        assertTrue(command is WaitUntilCommand)
    }

    @Test
    fun `instantCommand builds an InstantCommand that runs the given block`() {
        var ran = false
        val command = instantCommand { ran = true }

        assertTrue(command is InstantCommand)
        command.initialize()
        assertTrue(ran)
    }

    @Test
    fun `asInstantCommand wraps a lambda and requires no subsystems`() {
        var ran = false
        val toRun: () -> Unit = { ran = true }

        val command = toRun.asInstantCommand

        assertTrue(command is InstantCommand)
        assertEquals(0, command.requirements.size)
        command.initialize()
        assertTrue(ran)
    }
}
