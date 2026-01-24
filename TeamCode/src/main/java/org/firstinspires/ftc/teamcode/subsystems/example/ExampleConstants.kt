package org.firstinspires.ftc.teamcode.subsystems.example

import com.seattlesolvers.solverslib.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import org.firstinspires.ftc.teamcode.alonlib.units.AngularVelocity
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.rangeTo
import org.firstinspires.ftc.teamcode.alonlib.units.rpm

object ExampleConstants {
    /**
     * here you put any constant number relating to your subsystem
     */

    val MIN_COMPONENT_1 = 0.degrees

    val MAX_COMPONENT_1 = 90.degrees

    val MIN_COMPONENT_2 = 0.rpm

    val MAX_COMPONENT_2 = 6000.rpm

    class ExampleState(stateComponent1: Rotation2d, stateComponent2: AngularVelocity) {
        val stateComponent1: Rotation2d
        val stateComponent2: AngularVelocity

        init {
            this.stateComponent1 =
                if (stateComponent1.degrees in MIN_COMPONENT_1..MAX_COMPONENT_1) stateComponent1
                else {
                    robotPrintError("state component 1 is out of bounds $stateComponent1")
                    0.degrees
                }

            this.stateComponent2 =
                if (stateComponent2 in MIN_COMPONENT_2..MAX_COMPONENT_2) stateComponent2
                else {
                    robotPrintError("state component 2 is out of bounds $stateComponent2")
                    0.rpm
                }
        }

        companion object ExampleStates {
            var STATE1 = ExampleState(15.degrees, 2000.rpm)
            var STATE2 = ExampleState(30.degrees, 1500.rpm)
        }
    }

}