package ca.helios5009.hyperion.pathing

import ca.helios5009.hyperion.core.Motors
import ca.helios5009.hyperion.core.Movement
import ca.helios5009.hyperion.core.PIDFController
import ca.helios5009.hyperion.core.ProportionalController
import ca.helios5009.hyperion.hardware.Deadwheels
import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.hardware.Otos
import ca.helios5009.hyperion.misc.events.EventListener
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * HyperionPath is a class that allows for the creation of paths for the robot to follow.
 * Run the different commands to create a path for the robot to follow. The path will be
 * created by the movement object that is passed in the constructor.
 *
 * ALWAYS call start before running any other commands. AND call end after all other commands.
 *
 *
 * @param opMode The LinearOpMode object that is running the autonomous.
 * @param listener The EventListener object that is listening for events.
 * @param bot The Motors object that is controlling the motors.
 * @param tracking Either Otos or Deadwheels object that is tracking the robot's position.
 * @param debug A boolean that will enable debug mode if true.
 *
 * @see Movement
 * @see Otos
 * @see Deadwheels
 * @see PIDFController
 * @author Gilbert O.
*/
class PathBuilder<T: Odometry>(
	private val opMode: LinearOpMode,
	private val listener: EventListener,
	bot: Motors,
	tracking: T,
	private val debug: Boolean = false
) {
	enum class PathState {
		ENDED,
		RUNNING,
		WAITING
	}

	private val movement = Movement<T>(opMode, listener, bot, debug)
	private var state = PathState.ENDED

	init {
		movement.tracking = tracking
	}

	/**
	 * Start the path with the origin point.
	 * @param origin The origin point of the path.
	 */
	fun start(origin: Point): PathBuilder<T> {
		if (state == PathState.RUNNING) {
			throw IllegalStateException("Path is already running")
		}
		state = PathState.RUNNING
		listener.call(origin.event)
		movement.setPosition(origin)
		return this
	}

	/**
	 * Move the robot to a point.
	 * @param point The point to move to.
	 */
	@Deprecated("Only use Continuous")
	fun line(point: Point): PathBuilder<T> {
		when (state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting")
			}
			PathState.RUNNING -> {
				movement.goto(point)
			}
		}
		return this
	}

	/**
	 * Continuously move the robot to a point. With the list, the robot will calculate the path to the point.
	 * It will try to figure out speeds, but use the Point.useError() function to help with PID control.
	 * The .useError() will use that point as a way to control it's speed. Use this especially for making sharper turns.
	 *
	 * The way it calculates the speed is by using the point after the error point.
	 * It finds the total distance the bot has to travel and uses that to calculate the speed.
	 * It's not perfect, but it's a good way to get the bot to move.
	 *
	 * If you want control over turn speed, a good practice is to place down
	 *
	 * @param points The list of points to move to.
	 */
	fun segment(vararg points: Point): PathBuilder<T> {
		if (points.isEmpty()) {
			throw IllegalArgumentException("No points to move to")
		}
		when(state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting")
			}
			PathState.RUNNING -> {
				movement.run(points.asList())
			}
		}
		return this
	}

	/**
	 * Continuously move the robot to a point. With the list, the robot will calculate the path to the point.
	 * It will try to figure out speeds, but use the Point.useError() function to help with PID control.
	 * The .useError() will use that point as a way to control it's speed. Use this especially for making sharper turns.
	 *
	 * The way it calculates the speed is by using the point after the error point.
	 * It finds the total distance the bot has to travel and uses that to calculate the speed.
	 * It's not perfect, but it's a good way to get the bot to move.
	 *
	 * If you want control over turn speed, a good practice is to place down
	 *
	 * @param points The list of points to move to.
	 */
	fun segment(points: List<Point>): PathBuilder<T> {
		if (points.isEmpty()) {
			throw IllegalArgumentException("No points to move to")
		}
		when(state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting")
			}
			PathState.RUNNING -> {
				movement.run(points)
			}
		}
		return this
	}

	/**
	 * Alternate to end. End the path. This will stop the robot from moving. It will also call the event that is passed in.
	 * @param event The event to call when the path is done.
	 */
	fun endWithoutHold(event: String = "_") {
		when(state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting")
			}
			PathState.RUNNING -> {
				state = PathState.ENDED
				listener.call(event)
				movement.stopMovement()
			}
		}
	}

	/**
	 * This will hold the bot's position till the end of autonomous.
	 * Useful if there's a chance you get hit out of parking or something.
	 * @param event The event to call when the path is done.
	 */
	fun end(event: String = "_") {
		when (state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started / Path has already ended")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting")
			}
			PathState.RUNNING -> {
				state = PathState.ENDED
			}
		}
		val loopTime = if (debug) {
			ElapsedTime()
		} else {
			null
		}
		var totalLoopTime = 0.0
		var loopCount = 0
		listener.call(event)
		val currentPosition = movement.getPosition()
		while(opMode.opModeIsActive()) {
			val distance = movement.goto(currentPosition, true)
			if (debug) {
				val loopTimeValue = loopTime?.milliseconds() ?: 0.0
				totalLoopTime += loopTimeValue
				loopCount++
				opMode.telemetry.addLine("Waiting for Autonomous to end")
				opMode.telemetry.addLine("Distance: ${distance}in")
				opMode.telemetry.addLine("Loop : ${loopTimeValue}ms")
				opMode.telemetry.addLine("Average Loop Time: ${totalLoopTime / loopCount}ms")
				opMode.telemetry.update()
				loopTime?.reset()
			}
		}
		movement.stopMovement()
	}


	/**
	 * Wait for a certain amount of time. Bot will hold it's position as much as it can.
	 * This is useful for waiting for other bots to move.
	 * By holding it's position, any other bots that are pushing it will not affect it's position.
	 * @param time The time to wait in milliseconds.
	 * @param event The event to call when the time is up.
	 */
	fun wait(time: Double, event: String = "_"): PathBuilder<T> {
		when(state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting already")
			}
			PathState.RUNNING -> {
				state = PathState.WAITING
			}
		}

		val loopTime = if (debug) {
			ElapsedTime()
		} else {
			null
		}
		var totalLoopTime = 0.0
		var loopCount = 0
		listener.call(event)
		val currentPosition = movement.getPosition()
		val timer = ElapsedTime()
		while(opMode.opModeIsActive() && timer.milliseconds() < time) {
			val distance = movement.goto(currentPosition, true)
			if (debug) {
				val loopTimeValue = loopTime?.milliseconds() ?: 0.0
				totalLoopTime += loopTimeValue
				loopCount++
				opMode.telemetry.addLine("Waiting for ${time}ms")
				opMode.telemetry.addLine("Distance: ${distance}in")
				opMode.telemetry.addLine("Loop Time: ${loopTimeValue}ms")
				opMode.telemetry.addLine("Average Loop Time: ${totalLoopTime / loopCount}ms")
				opMode.telemetry.update()
				loopTime?.reset()
			}
		}
		state = PathState.RUNNING
		return this
	}

	/**
	 * Wait for a certain event to happen. Bot will hold it's position as much as it can.
	 * This is useful for waiting for event's to happen.
	 * By holding it's position, any other bots that are pushing it will not affect it's position.
	 * @param message The message to wait for.
	 * @param event The event to call when the message is called.
	 */
	fun wait(message: String, event: String = "_"): PathBuilder<T> {
		when(state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting already")
			}
			PathState.RUNNING -> {
				state = PathState.WAITING
			}
		}
		val loopTime = if (debug) {
			ElapsedTime()
		} else {
			null
		}
		var totalLoopTime = 0.0
		var loopCount = 0
		listener.call(event)
		val currentPosition = movement.getPosition()
		if (message.startsWith('_')) {
			val timer = ElapsedTime()
			while(opMode.opModeIsActive() && timer.milliseconds() < 1500.0) {
				val distance = movement.goto(currentPosition, true)

				if (debug) {
					val loopTimeValue = loopTime?.milliseconds() ?: 0.0
					totalLoopTime += loopTimeValue
					loopCount++
					opMode.telemetry.addLine("Waiting for 1500ms")
					opMode.telemetry.addLine("Distance: ${distance}in")
					opMode.telemetry.addLine("Loop Time: ${loopTimeValue}ms")
					opMode.telemetry.addLine("Average Loop Time: ${totalLoopTime / loopCount}ms")
					opMode.telemetry.update()
					loopTime?.reset()
				}
			}
		} else {
			while(opMode.opModeIsActive() && !listener.isInQueue(message)) {
				val distance = movement.goto(currentPosition, true)
				if (debug) {
					val loopTimeValue = loopTime?.milliseconds() ?: 0.0
					totalLoopTime += loopTimeValue
					loopCount++
					opMode.telemetry.addData("Waiting for", message)
					opMode.telemetry.addLine("Distance: ${distance}in")
					opMode.telemetry.addLine("Loop Time: ${loopTimeValue}ms")
					opMode.telemetry.addLine("Average Loop Time: ${totalLoopTime / loopCount}ms")
					opMode.telemetry.update()
					loopTime?.reset()
				}
			}
		}
		state = PathState.RUNNING
		return this
	}

	/**
	 * Get the current position of the bot.
	 * @return The current position of the bot.
	 */
	fun getVelocity(): Double {
		return movement.velocity
	}

	/**
	 * Get the acceleration of the bot.
	 * @return The acceleration of the bot.
	 */
	fun getAcceleration(): Double {
		return movement.acceleration
	}

	/*
	 * Get the distance from the current target point
	 */
	fun getDistanceFromTarget(): Double {
		return movement.distanceFromTarget.get()
	}

	/**
	 * Set the timeout for the end of segments.
	 * This timeout is used to make sure the bot doesn't get stuck in a segment when stuck trying to auto correct.
	 * @param time The time in milliseconds.
	 */
	fun setTimeout(time: Double) {
		movement.timeout = time
	}

	fun setDistanceTolerance(tolerance: Double) {
		movement.minimumVectorTolerance = tolerance
	}

	/**
	 * Set constants for the Drive PID controller
	 * @see PIDFController
	 */
	fun setDriveConstants(kP: Double, kI: Double, kD: Double, kF: Double, posTolerance: Double, velTolerance: Double) {
		movement.driveController = PIDFController(kP, kI, kD, kF)
		movement.driveController.setTolerance(posTolerance, velTolerance)
	}

	/**
	 * Set constants for the Strafe PID controller
	 * @see PIDFController
	 */
	fun setStrafeConstants(kP: Double, kI: Double, kD: Double, kF: Double, posTolerance: Double, velTolerance: Double) {
		movement.strafeController = PIDFController(kP, kI, kD, kF)
		movement.strafeController.setTolerance(posTolerance, velTolerance)
	}

	/**
	 * Set constants for the Rotation PID controller
	 *
	 * @see PIDFController
	 */
	fun setRotateConstants(kP: Double, kI: Double, kD: Double, kF: Double, posTolerance: Double, velTolerance: Double) {
		movement.rotateController = PIDFController(kP, kI, kD, kF)
		movement.rotateController.setTolerance(posTolerance, velTolerance)
	}

}