package ca.helios5009.hyperion.hardware

import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.pathing.Point
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Otos(
	hardwareMap: HardwareMap,
	name: String,
	offset: Point = Point(0.0, 0.0)
): Odometry {
	val otos = hardwareMap.get(name) as SparkFunOTOS

	var linearScalar: Double
		get() = otos.linearScalar
		set(value) { otos.linearScalar = value }

	var angularScalar: Double
		get() = otos.angularScalar
		set(value) { otos.angularScalar = value }

	var linearUnit: DistanceUnit
		get() = otos.linearUnit
		set(value) { otos.linearUnit = value }

	var angularUnit: AngleUnit
		get() = otos.angularUnit
		set(value) { otos.angularUnit = value }

	var offset: Point
		get() = Point(otos.offset.x, otos.offset.y).setRad(otos.offset.h)
		set(value) { otos.offset = SparkFunOTOS.Pose2D(value.x, value.y, value.rot) }

	val acceleration get() = Point(otos.acceleration.x, otos.acceleration.y)

	val velocity get() = Point(otos.velocity.x, otos.velocity.y)

	override var position: Point
		get() = Point(otos.position.x, otos.position.y).setRad(otos.position.h)
		set(value) { otos.position = SparkFunOTOS.Pose2D(value.x, value.y, value.rot) }

	init {
		otos.initialize()
		otos.linearUnit = DistanceUnit.INCH
		otos.angularUnit = AngleUnit.RADIANS

		otos.offset = SparkFunOTOS.Pose2D(offset.x, offset.y, offset.rot)
		otos.linearScalar = 1.0
		otos.angularScalar = 1.0

		otos.calibrateImu()
		otos.resetTracking()
	}

	fun calibrateImu() {
		otos.calibrateImu()
	}


}