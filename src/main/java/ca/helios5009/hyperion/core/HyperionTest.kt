package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.misc.FileReader
import ca.helios5009.hyperion.misc.Odometry
import ca.helios5009.hyperion.misc.events.EventListener
import ca.helios5009.hyperion.pathing.PathBuilder
import ca.helios5009.hyperion.pathing.Point
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import java.io.File

/**
 * HyperionTest is a class that reads a file and runs the commands in the file.
 * You can create a path through the Hyperion Controller app.
 *
 * Given a path, the HyperionTest will read the file and run the commands 1 by 1.
 *
 * @param opMode The LinearOpMode that the HyperionTest is running on.
 * @param listener The EventListener that the HyperionTest is using.
 * @param bot The Motors that the HyperionTest is using.
 * @param tracking The PositionTracking that the HyperionTest is using.
 * @param path The path to the file that contains the commands.
 * @constructor Create a new HyperionTest object.
 *
 * @see Movement
 * @see FileReader
 */
class HyperionTest<T: Odometry>(
	opMode: LinearOpMode,
	listener: EventListener,
	bot: Motors,
	tracking: T,
	path: String
) {
	private val pathBuilder = PathBuilder(opMode, listener, bot, tracking, true)
	private val file = File(path)

	fun run() {
		var isSegment = false
		var segmentLength: Byte = 0
		var currentSegmentPoint: Byte = 0
		val currentSegment = mutableListOf<Point>()

		for (line in file.readLines()) {
			if (line.startsWith('-')) {
				val stringCommand = line.substring(1)
				val splitCommand = stringCommand.split(" ").toMutableList()
				val command = splitCommand.removeFirst()
				val args = splitCommand.removeFirst()
				if (command == "end") {
					pathBuilder.end(args)
					break;
				}

				when (command) {
					"start" -> {
						val eventCall = splitCommand.removeFirst()
						val pointSplit = args.split(",")
						val point = Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), eventCall).setDeg(pointSplit[2].toDouble())
						pathBuilder.start(point)
					}
					"segment" -> {
						isSegment = true
						segmentLength = args.toByte()
					}
					"wait" -> {
						val eventCall = splitCommand.removeFirst()
						if (args[0].isDigit()) {
							pathBuilder.wait(convertTime(args), eventCall)
						} else {
							pathBuilder.wait(args, eventCall)
						}
					}
					else -> {
						throw Exception("Invalid command")
					}
				}
			} else if (isSegment) {
				if (segmentLength == 0.toByte()) {
					throw Exception("Invalid segment length")
				}
				val splitCommand = line.split(" ").toMutableList()
				var pointString = splitCommand.removeFirst()
				val tolerance = splitCommand.removeFirst()
				val eventCall = splitCommand.removeFirst()
				var usingError = false

				pointString = if (pointString.startsWith("*")) {
					usingError = true
					pointString.replace("*", "")
				} else {
					pointString
				}

				val pointSplit = pointString.split(",").toMutableList()
				val point = if (usingError) {
					Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), eventCall).setDeg(pointSplit[2].toDouble()).useError()
				} else {
					Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), eventCall).setDeg(pointSplit[2].toDouble())
				}

				if (!tolerance.startsWith('_') && tolerance != "0") {
					point.tolerance = tolerance.toDouble()
				}
				currentSegment.add(point)
				currentSegmentPoint++
				if (currentSegmentPoint == segmentLength) {
					pathBuilder.segment(currentSegment)
					currentSegmentPoint = 0
					currentSegment.clear()
					isSegment = false
				}
			}
		}
	}

	private fun convertTime(time: String): Double {
		var suffix = ""
		var timeString = ""
		for (char in time) {
			if (char.isDigit() || char == '.') {
				timeString += char
			} else {
				suffix += char
			}
		}

		return when (suffix) {
			"ms" -> {
				timeString.toDouble()
			}
			"s" -> {
				timeString.toDouble() * 1000
			}
			"m" -> {
				timeString.toDouble() * 60000
			}
			else -> {
				throw Exception("Invalid time")
			}
		}
	}
}