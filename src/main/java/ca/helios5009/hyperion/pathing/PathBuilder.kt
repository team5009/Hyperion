package ca.helios5009.hyperion.pathing

import ca.helios5009.hyperion.core.Movement
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * HyperionPath is a class that allows for the creation of paths for the robot to follow.
 * Run the different commands to create a path for the robot to follow. The path will be
 * created by the movement object that is passed in the constructor.
 *
 * ALWAYS call start before running any other commands. AND call end after all other commands.
 *
 * @param movement The Movement object that is used to move the robot.
 * @constructor Create a new HyperionPath object.
 *
 * @see Movement
 * @author Gilbert O.
*/
class PathBuilder(
	private val movement: Movement
) {
	/**
	 * Start the path with the origin point.
	 * @param origin The origin point of the path.
	 */
	fun start(origin: Point) {
		movement.listener.call(origin.event)
		movement.setPosition(origin)
	}

	/**
	 * Move the robot to a point.
	 * @param point The point to move to.
	 */
	@Deprecated("Only use Continuous")
	fun line(point: Point) {
		movement.run(listOf(point))
	}

//	@Suppress("Not implemented properly")
//	fun bezier(bezier: Bezier) {
//		val generatedBezier = generateBezier(bezier.start, bezier.control[0], bezier.control[1], bezier.end)
//		movement.run(generatedBezier)
//	}

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
	fun segment(vararg points: Point) {
		movement.run(points.asList())
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
	fun segment(points: List<Point>) {
		movement.run(points)
	}

	/**
	 * End the path. This will stop the robot from moving. It will also call the event that is passed in.
	 * @param event The event to call when the path is done.
	 */
	fun end(event: String = "_") {
		movement.listener.call(event)
		movement.stopMovement()
		movement.listener.clearQueue()
	}

	/**
	 * Wait for a certain amount of time. Bot will hold it's position as much as it can.
	 * This is useful for waiting for other bots to move.
	 * By holding it's position, any other bots that are pushing it will not affect it's position.
	 * @param time The time to wait in milliseconds.
	 * @param event The event to call when the time is up.
	 */
	fun wait(time: Double, event: String = "_") {
		val loopTime = if (movement.debug) {
			ElapsedTime()
		} else {
			null
		}
		var avgLoopTime = 0.0
		movement.listener.call(event)
		val currentPosition = movement.getPosition()
		val timer = ElapsedTime()
		while(movement.opMode.opModeIsActive() && timer.milliseconds() < time) {
			val distance = movement.goto(currentPosition, true)
			if (movement.debug) {
				val loopTimeValue = loopTime?.milliseconds() ?: 0.0
				avgLoopTime += loopTimeValue
				avgLoopTime /= 2
				movement.opMode.telemetry.addLine("Waiting for ${time}ms")
				movement.opMode.telemetry.addLine("Distance ${distance}in")
				movement.opMode.telemetry.addLine("Loop Time ${loopTimeValue}ms")
				movement.opMode.telemetry.addLine("Average Loop Time ${avgLoopTime}ms")
				movement.opMode.telemetry.update()
				loopTime?.reset()
			}
		}
	}

	/**
	 * Wait for a certain event to happen. Bot will hold it's position as much as it can.
	 * This is useful for waiting for event's to happen.
	 * By holding it's position, any other bots that are pushing it will not affect it's position.
	 * @param message The message to wait for.
	 * @param event The event to call when the message is called.
	 */
	fun wait(message: String, event: String = "_") {
		val loopTime = if (movement.debug) {
			ElapsedTime()
		} else {
			null
		}
		var avgLoopTime = 0.0
		movement.listener.call(event)
		val currentPosition = movement.getPosition()
		if (message.startsWith('_')) {
			val timer = ElapsedTime()
			while(movement.opMode.opModeIsActive() && timer.milliseconds() < 1500.0) {
				val distance = movement.goto(currentPosition, true)

				if (movement.debug) {
					val loopTimeValue = loopTime?.milliseconds() ?: 0.0
					avgLoopTime += loopTimeValue
					avgLoopTime /= 2
					movement.opMode.telemetry.addLine("Waiting for 1500ms")
					movement.opMode.telemetry.addLine("Distance ${distance}in")
					movement.opMode.telemetry.addLine("Loop Time ${loopTimeValue}ms")
					movement.opMode.telemetry.addLine("Average Loop Time ${avgLoopTime}ms")
					movement.opMode.telemetry.update()
					loopTime?.reset()
				}
			}
		} else {
			while(movement.opMode.opModeIsActive() && !movement.listener.isInQueue(message)) {
				val distance = movement.goto(currentPosition, true)
				if (movement.debug) {
					val loopTimeValue = loopTime?.milliseconds() ?: 0.0
					avgLoopTime += loopTimeValue
					avgLoopTime /= 2
					movement.opMode.telemetry.addData("Waiting for", message)
					movement.opMode.telemetry.addLine("Distance ${distance}in")
					movement.opMode.telemetry.addLine("Loop Time ${loopTimeValue}ms")
					movement.opMode.telemetry.addLine("Average Loop Time ${avgLoopTime}ms")
					movement.opMode.telemetry.update()
					loopTime?.reset()
				}
			}
		}
	}

}