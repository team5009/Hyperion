package ca.helios5009.hyperion.hardware

import ca.helios5009.hyperion.pathing.Point
import com.qualcomm.robotcore.hardware.DcMotorEx
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin



class Odometry (private val leftEncoder: DcMotorEx, private val rightEncoder: DcMotorEx, private val backEncoder: DcMotorEx) {

	var location = AtomicReference(Point(0.0, 0.0, 0.0))

	private var distanceLeftRight = 0.0
	private var distanceBack = 0.0
	private val encoderConstant: Double = PI * 2.0 / 2000.0

	private var currentLeft = 0.0
	private var currentRight = 0.0
	private var currentBack = 0.0

	private var lastLeft = 0.0
	private var lastRight = 0.0
	private var lastBack = 0.0

	fun setConstants(distanceBack: Double, distanceLeftRight: Double) {
		this.distanceBack = distanceBack
		this.distanceLeftRight = distanceLeftRight
	}

	fun calculate(): Point {
		lastBack = currentBack
		lastLeft = currentLeft
		lastRight = currentRight

		currentLeft = leftEncoder.currentPosition.toDouble()
		currentRight = -rightEncoder.currentPosition.toDouble()
		currentBack = backEncoder.currentPosition.toDouble()

		val deltaLeft = currentLeft - lastLeft
		val deltaRight = currentRight - lastRight
		val deltaBack = currentBack - lastBack

		val deltaTheta = ((deltaLeft - deltaRight) / distanceLeftRight) * encoderConstant
		val deltaX = ((deltaLeft + deltaRight) / 2.0) * encoderConstant
		val deltaY = deltaBack * encoderConstant - distanceBack * deltaTheta

		val current = location.get().clone()
		val theta = current.rot + deltaTheta
		current.x -= deltaX * cos(theta) + deltaY * sin(theta)
		current.y += deltaX * sin(theta) - deltaY * cos(theta)
		current.rot -= deltaTheta
		location.set(current)

		return current
	}

	fun getRotDegrees() : Double {
		return location.get().rot * 180 / (PI)
	}

	fun setOrigin (point: Point) {
		location.set(point)
	}

	fun getLocation() : Point {
		return location.get()
	}

}