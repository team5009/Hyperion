package ca.helios5009.hyperion.core

import android.annotation.SuppressLint
import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.misc.cosineLaw
import ca.helios5009.hyperion.misc.euclideanDistance
import ca.helios5009.hyperion.misc.events.EventListener
import ca.helios5009.hyperion.pathing.PathBuilder
import ca.helios5009.hyperion.pathing.Point
import ca.helios5009.hyperion.pathing.Segment
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

/**
 * Class that handles the movement of the robot
 *  - Set the constants for the [Movement.driveController], [Movement.strafeController], [Movement.rotateController] PID controllers
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
class Movement<T: Odometry>(
	private val opMode: LinearOpMode,
	private val listener: EventListener,
	private val bot: Motors,
	private val debug: Boolean
) {
	var minimumVectorTolerance: Double = 2.0
	var timeout = 150.0

	lateinit var tracking: T

	var velocity: Double = 0.0
	var acceleration: Double = 0.0

	lateinit var driveController : PIDFController
	lateinit var strafeController : PIDFController
	lateinit var rotateController : PIDFController

	private var finalPathPoint: Point = Point(0.0, 0.0)
	private var currentTargetPoint: Point = Point(0.0, 0.0)
	val segment = Segment()

	var currentPosition = Point(0.0, 0.0)
		private set
	private var previousPosition = Point(0.0, 0.0)

	private val kinematicsTimer: ElapsedTime = ElapsedTime() // Timer for calculating the velocity and acceleration
	private var previousVelocity: Double = 0.0
	private val loopTime = if (debug) {
		ElapsedTime()
	} else {
		null
	}
	private var totalLoopTime = 0.0
	private var loopCount = 0

	/**
	 * Run the path that is given to the robot.
	 * This method will move the robot to the points that are given in the path.
	 * @param points The list of points that the robot should move to
	 *
	 * @see Point
	 * @see PathBuilder
	 */
	@SuppressLint("DefaultLocale")
	fun run(points: List<Point>) {
		loopTime?.reset()
		segment.setPath(points)
		previousPosition = currentPosition // Set the previous position to the current position
		currentPosition = tracking.position // Get the current position of the robot
		while(segment.hasNext()) {
			segment.setHeading()
			val vectorTolerance = segment.calculateTolerance(currentPosition, minimumVectorTolerance)
			listener.call(segment.current.event)
			resetController()

			do {
				goto(segment.current)
				if (debug) {
					opMode.telemetry.addData("Current Execution", "Through Path")
					opMode.telemetry.addLine("Vector Tolerance: ${vectorTolerance}in")
					opMode.telemetry.addLine(String.format("Target Heading: %.2f", segment.lastKnownPosition.rot * 180 / Math.PI))
					opMode.telemetry.update()
				}
			} while (
				opMode.opModeIsActive() &&
				(
					segment.distanceFromTarget.get() > vectorTolerance ||
					abs(rotateController.positionError) >= rotateController.tolerances.first * 2.0
				)
			)

			segment.lastKnownPosition = segment.current
			segment.nextPoint() // Move to the next point
		}


		listener.call(finalPathPoint.event) // Call the event at the final point
		segment.setHeading()
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
	@SuppressLint("DefaultLocale")
	fun goto(targetPosition: Point, endPoint: Boolean = false) {
		currentPosition = tracking.position // Get the current position of the robot // Set the path index to the current point in the path
		calculateVelocity() // Calculate the velocity of the robot
		calculateAcceleration() // Calculate the acceleration of the robot
		kinematicsTimer.reset() // Reset the timer to calculate the velocity
		previousPosition = currentPosition.clone() // Set the previous position to the current position
		previousVelocity = velocity // Set the previous velocity to the current velocity

		val theta = currentPosition.rot// Get the current angle of the robot

		// Calculate the error between the target and the current position
		val error = currentPosition.distanceTo(targetPosition)
		segment.distanceFromTarget.set(error) // Set the distance from the target point

		// Calculate the magnitude (The total distance that the robot should go to remove the stutter between points)
		val magnitude = segment.lookForNextError(currentPosition)

		val deltaX = if (endPoint) (targetPosition.x - currentPosition.x) else
				(targetPosition.x - currentPosition.x) * magnitude / error

		val deltaY = if (endPoint) (targetPosition.y - currentPosition.y) else
			(targetPosition.y - currentPosition.y) * magnitude / error

		// Calculate the amount of drive error that the robot should move
		val driveError = deltaX * cos(-theta) - deltaY * sin(-theta)
		// Calculate the amount of strafe error that the robot should move
		val strafeError = deltaX * sin(-theta) + deltaY * cos(-theta)

		val drive  = driveController.directCalculate(driveError)
		val strafe = strafeController.directCalculate(strafeError)
		val rotate = rotateController.calculate(theta)
		bot.move(drive, -strafe, -rotate)
		if (debug) {
			val time = calculateLoopTime()
			opMode.telemetry.addLine(String.format("Drive: %.2f", drive))
			opMode.telemetry.addLine(String.format("Strafe: %.2f", strafe))
			opMode.telemetry.addLine(String.format("Rotate: %.2f", rotate))
			opMode.telemetry.addLine(String.format("Magnitude: %.2f in", magnitude,))
			opMode.telemetry.addLine("--------------------")
			opMode.telemetry.addLine(String.format("Distance: %.3f in", error))
			opMode.telemetry.addLine(String.format("Velocity: %.2f in/s", velocity))
			opMode.telemetry.addLine(String.format("Acceleration: %.2f in/s^2", acceleration))
			opMode.telemetry.addLine("--------------------")
			opMode.telemetry.addData("Position", currentPosition.toString())
			opMode.telemetry.addData("Target Point", currentTargetPoint.toString())
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
		resetController() // Reset the controllers
		val timeoutTimer = ElapsedTime() // Create a timer to timeout if the robot is stuck

		var inDrivePosition = false
		var inStrafePosition = false
		var inRotatePosition = false
		rotateController.setTarget(currentTargetPoint.rot) // Set the target for the rotate controller

		while (opMode.opModeIsActive()) {
			goto(targetPoint, true)
			if (debug) {
				if (driveController.isAtTarget && strafeController.isAtTarget && rotateController.isAtTarget ) {
					break
				}

				opMode.telemetry.addData("Current Execution", "To End Point")
				opMode.telemetry.update()
			} else {
				if (inDrivePosition && inStrafePosition && inRotatePosition) {
					if (
						timeoutTimer.milliseconds() > timeout
						|| (driveController.isAtTarget && strafeController.isAtTarget && rotateController.isAtTarget)
					) {
						break
					}
				} else {
					timeoutTimer.reset()
				}

				if (driveController.isAtTarget && !inDrivePosition) {
					inDrivePosition = true
				}
				if (strafeController.isAtTarget && !inStrafePosition) {
					inStrafePosition = true
				}
				if (rotateController.isAtTarget && !inRotatePosition) {
					inRotatePosition = true
				}
			}
		}
		segment.lastKnownPosition = targetPoint
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

	/**
	 * Reset the controllers.
	 * This usually is used to reset the controllers after the robot has reached the target point.
	 *
	 * @see PIDFController
	 */
	fun resetController() {
		driveController.reset()
		strafeController.reset()
		rotateController.reset()
	}

	fun stopMovement() {
		bot.stop()
	}


}