package org.firstinspires.ftc.teamcode.alonlib

import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedDegrees
import org.firstinspires.ftc.teamcode.alonlib.units.normalizedRotations
import org.junit.Test

class RunEnvironment {


	@Test
	fun run(){
		var position = 500.degrees

		println(position.normalizedDegrees)
	}
}
