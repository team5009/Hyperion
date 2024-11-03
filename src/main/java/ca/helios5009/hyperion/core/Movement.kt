package ca.helios5009.hyperion.core

import android.annotation.SuppressLint
import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.misc.cosineLaw
import ca.helios5009.hyperion.misc.euclideanDistance
import ca.helios5009.hyperion.misc.events.EventListener
import ca.helios5009.hyperion.pathing.PathBuilder
import ca.helios5009.hyperion.pathing.Point
import ca.helios5009.hyperion.pathing.PointType
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.cos
import kotlin.math.sin

/**
 * Class that handles the movement of the robot
 *  - Set the constants for the [Movement.setDriveConstants], [Movement.setStrafeConstants], [Movement.setRotateConstants] PID controllers
 *  - Set the [tracking] method for the robot and run [setTracking] to add the odometry calculation
 *  - [mimimun path tolerence][Movement.minimumVectorTolerance] for the robot to reach the point in a continuous path, how much leeway the robot has to reach the point
 *  - [timeout] when the robot is stuck, how long the robot should wait before moving on
 *
 *  Other methods include (Don't use this method directly, use [PathBuilder] methods instead):
 *  - [run] the path that is given to the robot
 *  - [goto] the point that is given to the robot
 *
 *  Debug Mode will show telemetry data on the robot's position, the current target point, the vector tolerance, the distance, and the loop time
 * @param opMode The LinearOpMode that the robot is running on
 * @param listener The ca.helios5009.hyperion.misc.events.EventListener that is used to call events
 * @param bot The Motors object that is used to move the robot
 * @param tracking The tracking method that is used to track the robot's position
 * @param debug If the debug telemetry should be shown
 * @constructor Create a new Movement object
 * @see Motors
 * @see EventListener
 * @see PathBuilder
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

	var distanceFromTarget = AtomicReference(0.0)
	var velocity: Double = 0.0
	var acceleration: Double = 0.0

	lateinit var driveController : ProportionalController
	lateinit var strafeController : ProportionalController
	lateinit var rotateController : ProportionalController


	private var finalPathPoint: Point = Point(0.0, 0.0, 0.0)
	private var currentTargetPoint: Point = Point(0.0, 0.0, 0.0)
	private var path: List<Point> = listOf()
	private var currentPathIndex = 0;

	private var currentPosition = Point(0.0, 0.0, 0.0)
	private var previousPosition = Point(0.0, 0.0, 0.0)


	private val kinematicsTimer: ElapsedTime = ElapsedTime() // Timer for calculating the velocity and acceleration
	private var previousVelocity: Double = 0.0

	/**
	 * Run the path that is given to the robot.
	 * This method will move the robot to the points that are given in the path.
	 * @param points The list of points that the robot should move to
	 *
	 * @see Point
	 * @see PathBuilder
	 */
	fun run(points: List<Point>) {
		path = points // Set the path to the list of points
		val finalPoint = points.last() // Get the final point in the path
		finalPathPoint = if (finalPoint.type == PointType.Global) {
			finalPoint
		} else {
			val relativePosition: Point = if (points.size - 2 < 0) {
				getPosition()
			} else {
				points[points.size - 2]
			}
			Point(
				finalPoint.x + relativePosition.x,
				finalPoint.y + relativePosition.y,
				finalPoint.rot + relativePosition.rot
			)
		}

		val loopTime = if (debug) {
			ElapsedTime()
		} else {
			null
		}
		var totalLoopTime = 0.0
		var loopCount = 0

		// Set the final path point to the last point in the list
		currentPathIndex = 0 // Set the current path index to 0
		while (currentPathIndex < path.size - 1) { // Loop through the path but leave the last point
			currentTargetPoint =
				points[currentPathIndex] // Set the current target point to the current point in the path
			previousPosition = currentPosition.clone() // Set the previous position to the current position
			currentPosition = getPosition() // Get the current position of the robot
			var angleOfPath = 0.0 // Initialize the angle of the path
			val vectorTolerance = if (currentTargetPoint.useManualTorence) {
				currentTargetPoint.getTolerance() // Use the manual tolerance if it is set
			} else {
				val pointAhead =
					path[currentPathIndex + 1] // Get the point that is ahead of the current point
				val distanceA = euclideanDistance(
					currentPosition,
					currentTargetPoint
				) // Calculate the distance between the current position and the target point
				val distanceB = euclideanDistance(
					currentTargetPoint,
					pointAhead
				) // Calculate the distance between the target point and the next point
				val distanceC = euclideanDistance(
					currentPosition,
					pointAhead
				) // Calculate the distance between the current position and the next point

				angleOfPath =
					cosineLaw(distanceA, distanceB, distanceC) // Calculate the angle of the path
				maxOf(
					minimumVectorTolerance,
					(Math.PI - angleOfPath) * minimumVectorTolerance
				) // Calculate the vector tolerance
			}

			listener.call(currentTargetPoint.event) // Call events
			resetController() // Reset the controllers
			do {
				val distance =
					goto(currentTargetPoint) // Move the robot closer to the target point and update the distance from the point
				distanceFromTarget.set(distance) // Set the distance from the target point
				if (debug) {
					val loopTimeValue = loopTime?.milliseconds() ?: 0.0
					totalLoopTime += loopTimeValue
					loopCount++
					opMode.telemetry.addData("Current Execution", "Through Path")
					opMode.telemetry.addData("Position", currentPosition.toString())
					opMode.telemetry.addData("Target Point", currentTargetPoint.toString())
					opMode.telemetry.addLine("Vector Tolerance: ${vectorTolerance}in")
					opMode.telemetry.addData("Angle of Path", angleOfPath)

					opMode.telemetry.addLine("--------------------")
					opMode.telemetry.addLine("Loop Time: ${loopTimeValue}ms")
					opMode.telemetry.addLine("Average Loop Time: ${totalLoopTime / loopCount}ms")
					opMode.telemetry.update()
					loopTime?.reset()
				}
			} while (
				opMode.opModeIsActive() &&
				distance > vectorTolerance
			) // Loop until the robot is within the vector tolerance
			currentPathIndex++ // Increment the path index
		}
		listener.call(finalPathPoint.event) // Call the event at the final point
		goToEndPoint() // Move the robot to the final point
		bot.stop()
		path = listOf()
	}

	/**
	 * Set power to the motors to move the robot to a point.
	 * Calculates the power needed to give to each motors (ONLY FOR [MECANUM](<en.wikipedia.org/wiki/Mecanum_wheel>) WHEELS)
	 * @param targetPosition The point to move the robot to
	 * @param endPoint If it is the end point
	 * @return The distance from the target point in inches
	 *
	 * @see Point
	 * @see ProportionalController.update
	 */
	@SuppressLint("DefaultLocale")
	fun goto(targetPosition: Point, endPoint: Boolean = false): Double {
		currentPosition = getPosition() // Get the current position of the robot // Set the path index to the current point in the path
		calculateVelocity() // Calculate the velocity of the robot
		calculateAcceleration() // Calculate the acceleration of the robot
		kinematicsTimer.reset() // Reset the timer to calculate the velocity
		previousPosition = currentPosition.clone() // Set the previous position to the current position
		previousVelocity = velocity // Set the previous velocity to the current velocity

		val theta = currentPosition.rot // Get the current angle of the robot

		// Calculate the error between the target and the current position
		val error = euclideanDistance(targetPosition, currentPosition)

		// Calculate the speed factor (The speed that the robot should go to remove the stutter between points)
		val speedFactor = if (endPoint) {
			error
		} else {
			lookForNextError(currentPosition)
		}

		val deltaX = (targetPosition.x - currentPosition.x) * speedFactor

		val deltaY = (targetPosition.y - currentPosition.y) * speedFactor

		val deltaRot = targetPosition.rot - theta

		// Calculate the amount of drive error that the robot should move
		val driveError = deltaX * cos(theta) - deltaY * sin(theta)
		// Calculate the amount of strafe error that the robot should move
		val strafeError = deltaX * sin(theta) + deltaY * cos(theta)

		val drive = driveController.update(driveError)
		val strafe = -strafeController.update(strafeError)
		val rotate = -rotateController.update(deltaRot)
		bot.move(drive, strafe, rotate)
		if (debug) {
			opMode.telemetry.addData("Drive", drive)
			opMode.telemetry.addData("Strafe", strafe)
			opMode.telemetry.addData("Rotate", rotate)
			opMode.telemetry.addData("Speed Factor", speedFactor)
			opMode.telemetry.addLine("--------------------")
			opMode.telemetry.addLine("Distance: ${error}in")
			opMode.telemetry.addLine(String.format("Velocity: %.2f in/s", velocity))
			opMode.telemetry.addLine(String.format("Acceleration: %.2f in/s^2", acceleration))
			opMode.telemetry.addLine("--------------------")
		}
		return error
	}

	/**
	 * Move the robot to the end point.
	 * This is used to make sure that the robot reaches the end point.
	 * It will keep moving as long as one of the controllers have been to the ready position.
	 *
	 * @see Point
	 * @see ProportionalController
	 */
	private fun goToEndPoint() {
		val timeoutTimer = ElapsedTime() // Create a timer to timeout if the robot is stuck
		val loopTime = if (debug) {
			ElapsedTime()
		} else {
			null
		}
		var totalLoopTime = 0.0
		var loopCount = 0

		var inDrivePosition = false
		var inStrafePosition = false
		var inRotatePosition = false
		while (opMode.opModeIsActive()) {
			val distance = goto(finalPathPoint, true)
			distanceFromTarget.set(distance)
			if (debug) {
				if ( driveController.inPosition && strafeController.inPosition && rotateController.inPosition ) {
					break
				}
				val loopTimeValue = loopTime?.milliseconds() ?: 0.0
				totalLoopTime += loopTimeValue
				loopCount++
				opMode.telemetry.addData("Current Execution", "To End Point")
				opMode.telemetry.addData("Position", currentPosition.toString())
				opMode.telemetry.addData("Target Point", finalPathPoint.toString())
				opMode.telemetry.addLine("Distance: ${distance}in")
				opMode.telemetry.addLine("--------------------")
				opMode.telemetry.addLine("Loop Time: ${loopTimeValue}ms")
				opMode.telemetry.addLine("Average Loop Time: ${loopTimeValue / loopCount}ms")

				opMode.telemetry.update()
				loopTime?.reset()
			} else {
				if (inDrivePosition && inStrafePosition && inRotatePosition) {
					if (
						timeoutTimer.milliseconds() > timeout
						|| (driveController.inPosition && strafeController.inPosition && rotateController.inPosition)
					) {
						break
					}
				} else {
					timeoutTimer.reset()
				}

				if (driveController.inPosition && !inDrivePosition) {
					inDrivePosition = true
				}
				if (strafeController.inPosition && !inStrafePosition) {
					inStrafePosition = true
				}
				if (rotateController.inPosition && !inRotatePosition) {
					inRotatePosition = true
				}
			}

		}
	}

	/**
	 * Look for the next error in the path. This is used to calculate the speed factor
	 * When looking for the next error, it use's the point after the error point to control it's distance.
	 * Best practice to set point that will act as a reference point after the error if you want to control the speed sooner or later.
	 * @param position The current position of the robot
	 * @return The distance to the next error
	 *
	 * @see Point
	 */
	private fun lookForNextError(position: Point): Double {
		var distance = 0.0
		var point = position // initialize with position

		for (i in currentPathIndex until path.size) { // Range
			val nextPoint = path[i] // Get the next point in the path
			val error = euclideanDistance(nextPoint, point) // Calculate the error between the next point and the current point
			distance += error // Add the error to the distance
			point = nextPoint // Set the current point to the next point
			if (point.useError) { // Check if the point uses error and if it is not the end point
				val nextNextPoint = path[i + 1] // Get the next point after the error point
				val nextError = euclideanDistance(nextNextPoint, point) // Calculate the error between the next point and the current point
				distance += nextError // Add the error to the distance
				break
			}
		}

		return distance
	}

	fun getPosition(): Point {
		return tracking.getPosition()
	}

	fun setPosition(point: Point) {
		tracking.setPosition(point)
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

	private fun resetController() {
		driveController.reset()
		strafeController.reset()
		rotateController.reset()
	}

	fun stopMovement() {
		bot.stop()
	}


}