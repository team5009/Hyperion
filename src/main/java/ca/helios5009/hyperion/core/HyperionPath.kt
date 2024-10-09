package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.misc.commands.Bezier
import ca.helios5009.hyperion.misc.commands.EventCall
import ca.helios5009.hyperion.misc.commands.Point
import ca.helios5009.hyperion.misc.events.EventListener
import ca.helios5009.hyperion.misc.generateBezier
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * HyperionPath is a class that allows for the creation of paths for the robot to follow.
 * Run the different commands to create a path for the robot to follow. The path will be
 * created by the movement object that is passed in the constructor.
 *
 * ALWAYS call start before running any other commands. AND call end after all other commands.
 *
 * @param opMode The LinearOpMode that the robot is running on.
 * @param listener The EventListener that is used to call events.
 * @param movement The Movement object that is used to move the robot.
 * @constructor Create a new HyperionPath object.
 *
 * @see Movement
 * @author Gilbert O.
*/
class HyperionPath(
	private val opMode: LinearOpMode,
	val movement: Movement
) {
	/**
	 * Start the path with the origin point.
	 * @param origin The origin point of the path.
	 */
	fun start(origin: Point) {
		movement.listener.call(origin.event.message)
		movement.setPosition(origin)
	}

	/**
	 * Move the robot to a point.
	 * @param point The point to move to.
	 */
	fun line(point: Point) {
		movement.run(mutableListOf(point))
	}

	fun bezier(bezier: Bezier) {
		val generatedBezier = generateBezier(bezier.start, bezier.control[0], bezier.control[1], bezier.end)
		movement.run(generatedBezier)
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
	fun continuousLine(points: List<Point>) {
		movement.run(points)
	}

	/**
	 * End the path. This will stop the robot from moving. It will also call the event that is passed in.
	 * @param event The event to call when the path is done.
	 */
	fun end(event: EventCall) {
		movement.listener.call(event.message)
		movement.stopMovement()
	}

	/**
	 * Wait for a certain amount of time. Bot will hold it's position as much as it can.
	 * This is useful for waiting for other bots to move.
	 * By holding it's position, any other bots that are pushing it will not affect it's position.
	 * @param time The time to wait in milliseconds.
	 * @param event The event to call when the time is up.
	 */
	fun wait(time: Double, event: EventCall) {
		movement.listener.call(event.message)
		val currentPosition = movement.getPosition()
		val timer = ElapsedTime()
		while(movement.opMode.opModeIsActive() && timer.milliseconds() < time) {
			movement.goto(currentPosition, true)
		}
	}

	/**
	 * Wait for a certain event to happen. Bot will hold it's position as much as it can.
	 * This is useful for waiting for event's to happen.
	 * By holding it's position, any other bots that are pushing it will not affect it's position.
	 * @param message The message to wait for.
	 * @param event The event to call when the message is called.
	 */
	fun wait(message: String, event: EventCall) {
		movement.listener.call(event.message)
		val currentPosition = movement.getPosition()
		while(movement.opMode.opModeIsActive() && !movement.listener.isInQueue(message)) {
			movement.goto(currentPosition, true)
		}
	}

}