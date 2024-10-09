package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.misc.commands.EventCall
import ca.helios5009.hyperion.misc.commands.Point
import java.io.File

class HyperionTest(movement: Movement, private val path: String) {
	val HyperionPath = HyperionPath(movement.opMode, movement)


	fun run() {
		val file = File(path)
		var isContinuous = false
		var continousLength: Byte = 0
		var currentContinousPoint: Byte = 0
		val currentPointList = mutableListOf<Point>()

		for (line in file.readLines()) {
			if (line.startsWith('-')) {
				val stringCommand = line.substring(1)
				val splitCommand = stringCommand.split(" ").toMutableList()
				val command = splitCommand.removeFirst()
				val args = splitCommand.removeFirst()
				if (command == "end") {
					HyperionPath.end(EventCall(args))
					break;
				}
				val eventCall = splitCommand.removeFirst()
				when (command) {
					"start" -> {
						val pointSplit = args.split(",")
						val point = Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), pointSplit[2].toDouble(), EventCall(eventCall))
						HyperionPath.start(point)
					}
					"continuous" -> {
						isContinuous = true
						continousLength = args.toByte()
					}
					"wait" -> {
						if (args[0].isDigit()) {
							HyperionPath.wait(convertTime(args), EventCall(eventCall))
						} else {
							HyperionPath.wait(args, EventCall(eventCall))
						}
					}
					else -> {
						throw Exception("Invalid command")
					}
				}
			} else if (isContinuous) {
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
					Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), pointSplit[2].toDouble()).useError()
				} else {
					Point(pointSplit[0].toDouble(), pointSplit[1].toDouble(), pointSplit[2].toDouble())
				}

				if (!tolerance.startsWith('_') && tolerance != "0") {
					point.setTolerence(tolerance.toDouble())
				}
				currentPointList.add(point)
				currentContinousPoint++
				if (currentContinousPoint == continousLength) {
					HyperionPath.continuousLine(currentPointList)
					currentContinousPoint = 0
					currentPointList.clear()
					isContinuous = false
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

	enum class Command {
		START,
		END,
		CONTINUOUS,
		WAIT,
		NONE
	}
}