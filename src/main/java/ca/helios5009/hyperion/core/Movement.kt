package ca.helios5009.hyperion.core

import android.annotation.SuppressLint
import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.misc.euclideanDistance
import ca.helios5009.hyperion.misc.events.EventListener
import ca.helios5009.hyperion.pathing.PathBuilder
import ca.helios5009.hyperion.pathing.Point
import ca.helios5009.hyperion.pathing.Segment
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Class that handles the movement of the robot
 *  - Set the constants for the [Movement.xController], [Movement.yController], [Movement.rotController] PID controllers
 *  - Set the [tracking] method for the robot and run to add the odometry calculation
 *  - [mimimun path tolerence][Movement.minimumVectorTolerance] for the robot to reach the point in a continuous path, how much leeway the robot has to reach the point
 *  - [timeout] when the robot is stuck, how long the robot should wait before moving on
 *
 *  Other methods include (Don't use this method directly, use [PathBuilder] methods instead):
 *  - [run] the path that is given to the robot
 *  - [goto] the point that is given to the robot
 *
 *  Debug Mode will show telemetry data on the robot's position, the current target point, the vector tolerance, the distance, and the loop time
 * @param opMode [LinearOpMode] that the robot is running on
 * @param listener [EventListener] that is used to call events
 * @param bot [Motors] object that is used to move the robot
 * @param debug If the debug [org.firstinspires.ftc.robotcore.external.Telemetry] should be shown
 * @constructor Create a new Movement object
 * @see Motors
 * @see EventListener
 * @see PathBuilder
 * @see PIDFController
 */
@SuppressLint("DefaultLocale")
class Movement<T: Odometry>(
	private val opMode: LinearOpMode,
	private val listener: EventListener,
	private val bot: Motors,
	private val debug: Boolean
) {
	var minimumVectorTolerance: Double = 2.0
	var timeout = 150.0

	var velocity: Double = 0.0
	var acceleration: Double = 0.0

	lateinit var tracking: T

	lateinit var xPIDFController: PIDFController
	lateinit var yPIDFController: PIDFController
	lateinit var rotController : PIDFController

	val segment = Segment()

	val currentPosition = Point(0.0, 0.0).setRad(0.0)
	private val previousPosition = Point(0.0, 0.0).setRad(0.0)

	private val kinematicsTimer: ElapsedTime = ElapsedTime() // Timer for calculating the velocity and acceleration
	private var previousVelocity: Double = 0.0
	private val loopTime = if (debug) ElapsedTime() else null

	private var totalLoopTime = 0.0
	private var loopCount = 0

	/**
	 *
	 * @param points The list of points that the robot should move to
	 *
	 * @see Point
	 * @see PIDFController
	 */
	fun run(points: List<Point>) {
		loopTime?.reset()
		segment.setPath(points)
		previousPosition.set(currentPosition) // Set the previous position to the current position
		currentPosition.set(tracking.position) // Get the current position of the robot
		while(segment.hasNext()) {
//			segment.setHeading(segment.current)
			val vectorTolerance = segment.calculateTolerance(currentPosition, minimumVectorTolerance)
			listener.call(segment.current.event)
			resetController()
			rotController.setTarget(segment.current.rot)
			setTargetControllers(segment.current)
			do {
				goto(segment.current)
				if (debug) {
					opMode.telemetry.addData("Current Execution", "Through Path")
					opMode.telemetry.addLine("Vector Tolerance: ${vectorTolerance}in")
					opMode.telemetry.update()
				}
			} while (opMode.opModeIsActive() &&
				(
					segment.distanceFromTarget.get() > vectorTolerance ||
					abs(rotController.positionError) >= rotController.tolerances.first * 2.0
				)
			)
			segment.setLastKnownPosition(segment.current) // Set the last known position to the current position
			segment.nextPoint() // Move to the next point
		}

		listener.call(segment.last.event) // Call the event at the final point
		resetController() // Reset the controllers
		goToEndPoint(segment.last) // Move the robot to the final point
		bot.stop()
		segment.clear()
	}

	/**
	 * Set power to the motors to move the robot to a point.
	 * Calculates the power needed to give to each motors (ONLY FOR [MECANUM](<en.wikipedia.org/wiki/Mecanum_wheel>) WHEELS)
	 * @param targetPosition The point to move the robot to
	 * @param endPoint If it is the end point
	 * @return The distance from the target point in inches
	 *
	 * @see Point
	 * @see PIDFController
	 */

	fun goto(targetPosition: Point, endPoint: Boolean = false) {
		currentPosition.set(tracking.position) // Get the current position of the robot // Set the path index to the current point in the path
		calculateVelocity() // Calculate the velocity of the robot
		calculateAcceleration() // Calculate the acceleration of the robot
		kinematicsTimer.reset() // Reset the timer to calculate the velocity
		previousPosition.set(currentPosition) // Set the previous position to the current position
		previousVelocity = velocity // Set the previous velocity to the current velocity

		val theta = currentPosition.rot// Get the current angle of the robot

		val error = currentPosition.distanceTo(targetPosition) // Calculate the distance between the target and the current position
		segment.distanceFromTarget.set(error) // Set the distance from the target point

		// Calculate the magnitude (The total distance that the robot should go to remove the stutter between points)
		val magnitude = segment.lookForNextError(currentPosition)

		val deltaX = if (endPoint) xPIDFController.directCalculate(currentPosition.x) else
			xPIDFController.directCalculate(currentPosition.x) * magnitude / error

		val deltaY = if (endPoint) yPIDFController.directCalculate(currentPosition.y) else
			yPIDFController.directCalculate(currentPosition.y) * magnitude / error

		// Calculate the amount of drive error that the robot should move
		val drive = deltaX * cos(-theta) - deltaY * sin(-theta)
		// Calculate the amount of strafe error that the robot should move
		val strafe = deltaX * sin(-theta) + deltaY * cos(-theta)
		val rotate = rotController.directCalculate(theta)

		bot.move(drive, -strafe, -rotate)
		if (debug) {
			val time = calculateLoopTime()
			opMode.telemetry.addLine(String.format("Drive: %.2f", drive))
			opMode.telemetry.addLine(String.format("Strafe: %.2f", strafe))
			opMode.telemetry.addLine(String.format("Rotate: %.2f", rotate))
			opMode.telemetry.addLine(String.format("Magnitude: %.2f in", magnitude))
			opMode.telemetry.addLine("--------------------")
			opMode.telemetry.addLine(String.format("Distance: %.3f in", error))
			opMode.telemetry.addLine(String.format("Velocity: %.2f in/s", velocity))
			opMode.telemetry.addLine(String.format("Acceleration: %.2f in/s^2", acceleration))
			opMode.telemetry.addLine("--------------------")
			opMode.telemetry.addData("Position", currentPosition.toString())
			opMode.telemetry.addData("Target Point", targetPosition.toString())
			opMode.telemetry.addData("Last Known Set:", segment.lastKnownPosition.toString())
			opMode.telemetry.addLine("--------------------")
			opMode.telemetry.addLine(String.format("Loop Time: %.2f ms", time.first))
			opMode.telemetry.addLine(String.format("Average Loop Time: %.2f ms", time.second))
		}
	}

	/**
	 * Move the robot to the end point.
	 * This is used to make sure that the robot reaches the end point.
	 * It will keep moving as long as one of the controllers have been to the ready position.
	 *
	 * @see Point
	 * @see PIDFController
	 */
	fun goToEndPoint(targetPoint: Point) {
		val timeoutTimer = ElapsedTime() // Create a timer to timeout if the robot is stuck
		var inDrivePosition = false; var inStrafePosition = false; var inRotatePosition = false
		setTargetControllers(targetPoint)
		while (opMode.opModeIsActive()) {
			goto(targetPoint, true)
			if (debug) {
				if (xPIDFController.isAtTarget && yPIDFController.isAtTarget && rotController.isAtTarget) break
				opMode.telemetry.addData("Current Execution", "To End Point")
				opMode.telemetry.update()
			} else {
				if (inDrivePosition && inStrafePosition && inRotatePosition)
					if ((xPIDFController.isAtTarget && yPIDFController.isAtTarget && rotController.isAtTarget) ||
						timeoutTimer.milliseconds() > timeout) break else timeoutTimer.reset()

				if (xPIDFController.isAtTarget && !inDrivePosition) inDrivePosition = true
				if (yPIDFController.isAtTarget && !inStrafePosition) inStrafePosition = true
				if (rotController.isAtTarget && !inRotatePosition) inRotatePosition = true
			}
		}
		segment.setLastKnownPosition(targetPoint)
	}

	private fun calculateVelocity() {
		val deltaTime = kinematicsTimer.seconds()
		val deltaPosition = euclideanDistance(currentPosition, previousPosition)
		velocity = deltaPosition / deltaTime
	}

	private fun calculateAcceleration() {
		val deltaTime = kinematicsTimer.seconds()
		val deltaVelocity = velocity - previousVelocity
		acceleration = deltaVelocity / deltaTime
	}

	/**
	 * Calculate the loop time and the average loop time.
	 * This is used to calculate the loop time and the average loop time.
	 * @return The loop time and the average loop time
	 */
	private fun calculateLoopTime(): Pair<Double, Double> {
		val loopTimeValue = loopTime?.milliseconds() ?: 0.0
		totalLoopTime += loopTimeValue
		loopCount++

		loopTime?.reset()
		return Pair(loopTimeValue, totalLoopTime / loopCount)
	}

	fun initStart(start: Point) {
		currentPosition.set(start)
		previousPosition.set(start)
		tracking.position = start
		segment.setLastKnownPosition(start)
		resetController()
		loopTime?.reset()
		stopMovement()
	}

	/**
	 * Reset the controllers.
	 * This usually is used to reset the controllers after the robot has reached the target point.
	 *
	 * @see PIDFController
	 */
	private fun resetController() {
		xPIDFController.reset()
		yPIDFController.reset()
		rotController.reset()
	}

	fun setTargetControllers(target: Point) {
		xPIDFController.setTarget(target.x)
		yPIDFController.setTarget(target.y)
		rotController.setTarget(target.rot)
	}

	fun stopMovement() {
		bot.stop()
	}


}