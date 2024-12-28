package ca.helios5009.hyperion.pathing

import ca.helios5009.hyperion.core.Motors
import ca.helios5009.hyperion.core.Movement
import ca.helios5009.hyperion.core.PIDFController
//import ca.helios5009.hyperion.hardware.Deadwheels
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
	private val bot: Motors,
	private val tracking: T,
	private val debug: Boolean = false
) {
	enum class PathState {
		ENDED,
		RUNNING,
		WAITING
	}

	/**
	 * The current position of the bot.
	 */
	var position get() = tracking.position
		private set(value) {
			tracking.position = value
			movement.lastKnownPoint = value
			if (value.angleSet) {
				movement.lastKnowRotation = value.rot
			}
		}

	/**
	 * The current velocity of the bot.
	 */
	val acceleration get() = movement.acceleration

	/**
	 * The current velocity of the bot.
	 */
	val velocity get() = movement.velocity

	/**
	 * The distance from the current target point.
	 */
	val distanceFromTarget get() = movement.distanceFromTarget.get()

	/**
	 * The max speed of the bot.
	 */
	var maxSpeed = 1.0
		set(value) {
			if (maxSpeed <= 0 && maxSpeed > 1) {
				throw IllegalArgumentException("Max speed must be between 0 and 1")
			}
			field = value
			bot.setPowerRatio(value)
		}

	private val movement = Movement<T>(opMode, listener, bot, debug)
	var state = PathState.ENDED
		private set
	private var holdingPosition = Point(0.0, 0.0).setRad(0.0)
		set(value) {
			field = value
			if (value.angleSet) {
				movement.lastKnowRotation = value.rot
			}
		}

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
		if(!origin.angleSet) {
			throw IllegalArgumentException("Origin point must have an angle set")
		}
		state = PathState.RUNNING
		listener.call(origin.event)
		position = origin
		return this
	}

	/**
	 * Move the robot to a point.
	 * @param point The point to move to.
	 */
	@Deprecated("Only use Segments or relativeLine")
	fun line(point: Point): PathBuilder<T> {
		when (state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting")
			}
			PathState.RUNNING -> {
				movement.run(listOf(point))
			}
		}
		return this
	}

	/**
	 * Move the robot to a point relative to the current position.
	 * @param point How much to move the robot.
	 */
	fun relativeLine(point: Point, powerRatio: Double = 1.0): PathBuilder<T> {
		when (state) {
			PathState.ENDED -> {
				throw IllegalStateException("Path has not started")
			}
			PathState.WAITING -> {
				throw IllegalStateException("Path is waiting")
			}
			PathState.RUNNING -> {

			}
		}

		holdingPosition = (movement.lastKnownPoint + point)
		bot.setPowerRatio(powerRatio)
		listener.call(point.event)
		movement.goToEndPoint(holdingPosition)
		bot.setPowerRatio(1.0)

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
		val currentPosition = movement.currentPosition
		while(opMode.opModeIsActive()) {
			movement.goto(currentPosition, true)
			if (debug) {
				opMode.telemetry.addLine("Waiting for Autonomous to end")
				opMode.telemetry.update()

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
		listener.call(event)
		holdingPosition = movement.lastKnownPoint
		val timer = ElapsedTime()
		while(opMode.opModeIsActive() && timer.milliseconds() < time) {
				movement.goto(holdingPosition, true)
				opMode.telemetry.addLine("Waiting for ${time}ms")
				opMode.telemetry.update()
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

		listener.call(event)
		holdingPosition = movement.lastKnownPoint
		if (message.startsWith('_')) {
			val timer = ElapsedTime()
			while(opMode.opModeIsActive() && timer.milliseconds() < 1500.0) {
				movement.goto(holdingPosition, true)

				if (debug) {
					opMode.telemetry.addLine("Waiting for 1500ms")
					opMode.telemetry.update()
				}
			}
		} else {
			while(opMode.opModeIsActive() && !listener.isInQueue(message)) {
				movement.goto(holdingPosition, true)
				if (debug) {
					opMode.telemetry.addData("Waiting for", message)
					opMode.telemetry.update()
				}
			}
		}
		state = PathState.RUNNING
		return this
	}

	fun moveHoldPosition(point: Point) {
		holdingPosition = holdingPosition + point
		movement.lastKnownPoint = point
	}

	/**
	 * Set the timeout for the end of segments.
	 * This timeout is used to make sure the bot doesn't get stuck in a segment when stuck trying to auto correct.
	 * @param time The time in milliseconds.
	 */
	fun setTimeout(time: Double): PathBuilder<T> {
		movement.timeout = time
		return this
	}

	fun setDistanceTolerance(tolerance: Double): PathBuilder<T> {
		movement.minimumVectorTolerance = tolerance
		return this
	}


	/**
	 * Set constants for the Drive PID controller
	 * @see PIDFController
	 */
	fun setDriveConstants(kP: Double, kI: Double, kD: Double, posTolerance: Double, velTolerance: Double): PathBuilder<T> {
		movement.driveController = PIDFController(kP, kI, kD,0.0)
		movement.driveController.setTolerance(posTolerance, velTolerance)
		return this
	}

	/**
	 * Set constants for the Strafe PID controller
	 * @see PIDFController
	 */
	fun setStrafeConstants(kP: Double, kI: Double, kD: Double, posTolerance: Double, velTolerance: Double): PathBuilder<T> {
		movement.strafeController = PIDFController(kP, kI, kD, 0.0)
		movement.strafeController.setTolerance(posTolerance, velTolerance)
		return this
	}

	/**
	 * Set constants for the Rotation PID controller
	 *
	 * @see PIDFController
	 */
	fun setRotateConstants(kP: Double, kI: Double, kD: Double, kF: Double, posTolerance: Double, velTolerance: Double): PathBuilder<T> {
		movement.rotateController = PIDFController(kP, kI, kD, kF)
		movement.rotateController.setTolerance(posTolerance, velTolerance)
		movement.rotateController.setAngleWrapAround()
		return this
	}

}