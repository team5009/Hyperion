package ca.helios5009.hyperion.hardware

import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.pathing.Point
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin



class Deadwheels (private val lateralEncoder: DcMotorEx, private val axialEncoder: DcMotorEx, private val imu: IMU):
	Odometry {

	private var location = Point(0.0, 0.0, 0.0)

	private val encoderConstant: Double = PI * 2.0 / 2000.0
	private var lateralOffset = 0.0
	private var axialOffset = 0.0

	private var currentLateral = 0.0
	private var currentAxial = 0.0

	private var lastLateral = 0.0
	private var lastAxial = 0.0

	/**
	 * Set the offsets for the lateral and axial encoders.
	 * @param lateralOffset The offset for the lateral encoder. Measure (inches) distance from center to the forward encoder.
	 * @param axialOffset The offset for the axial encoder. Measure (inches) distance from center to the sideways encoder.
	 */
	fun setConstants(lateralOffset: Double, axialOffset: Double) {
		this.lateralOffset = lateralOffset
		this.axialOffset = axialOffset
	}

	override fun getPosition(): Point {
		val rot = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

		lastLateral = currentLateral
		lastAxial = currentAxial

		currentLateral = lateralEncoder.currentPosition.toDouble()
		currentAxial = axialEncoder.currentPosition.toDouble()

		val deltaLateral = currentLateral - lastLateral
		val deltaAxial = currentAxial - lastAxial


		val deltaX = (deltaLateral * encoderConstant) - lateralOffset
		val deltaY = (deltaAxial * encoderConstant) - axialOffset

		location.rot = rot
		location.x += deltaX * cos(rot) - deltaY * sin(rot)
		location.y += deltaX * sin(rot) + deltaY * cos(rot)
		return location
	}

	override fun setPosition(point: Point) {
		location = point
	}

}