package ca.helios5009.hyperion.misc

import ca.helios5009.hyperion.misc.commands.Point
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Otos(
	hardwareMap: HardwareMap,
	name: String,
	offset: Point,
) {
	val otos = hardwareMap.get(name) as SparkFunOTOS

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

	fun resetTracking() {
		otos.resetTracking()
	}

	fun getPosition(): Point {
		val pose = otos.position
		return Point(pose.x, pose.y, pose.h)
	}

	fun setPosition(point: Point) {
		otos.position = SparkFunOTOS.Pose2D(point.x, point.y, point.rot)
	}

	fun setLinearScalar(scalar: Double) {
		otos.linearScalar = scalar
	}

	fun setAngularScalar(scalar: Double) {
		otos.angularScalar = scalar
	}

	fun setLinearUnit(unit: DistanceUnit) {
		otos.linearUnit = unit
	}

	fun setAngularUnit(unit: AngleUnit) {
		otos.angularUnit = unit
	}

	fun setOffset(offset: Point) {
		otos.offset = SparkFunOTOS.Pose2D(offset.x, offset.y, offset.rot)
	}

	fun calibrateImu() {
		otos.calibrateImu()
	}

	fun getVelocity(): Point {
		val velocity = otos.velocity
		return Point(velocity.x, velocity.y, velocity.h)
	}

	fun getAcceleration(): Point {
		val acceleration = otos.acceleration
		return Point(acceleration.x, acceleration.y, acceleration.h)
	}

}