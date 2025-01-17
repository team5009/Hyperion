package ca.helios5009.hyperion.core

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.sign

/**
 * Closed loop Proportional Controller
 * @see PIDFController
 */
@Deprecated("Use PIDFController")
class ProportionalController(
	private val gain: Double,
	private val accelLimit: Double,
	private val tolerance: Double,
	private val deadband: Double,
	private val circular: Boolean = false
) {
	var inPosition = false

	private var lastOutput = 0.0
	private var cycleTime = ElapsedTime()

	init {
		reset(true)
	}

	/**
	 * Determines power required to obtain the desired set point value based on new input value.
	 * Uses proportional gain, and limits rate of change of output, as well as max output.
	 * Make sure deadband is set to be smaller than the tolerance.
	 * @param inputError  Current live control input value (from sensors)
	 * @return desired output power.
	 * @see Movement.goto
	 */
	fun update(inputError: Double): Double {
		var error = inputError
		val absError = abs(error)
		val dV = cycleTime.seconds() * accelLimit // limit acceleration

		// normalize to +/- PI if we are controlling heading
		if (circular) {
			while (error > Math.PI) error -= 2 * Math.PI
			while (error <= -Math.PI) error += 2 * Math.PI
		}
		inPosition = absError < tolerance
		// Prevent any very slow motor output accumulation
		val output = if (absError <= deadband) {
			0.0
		} else {
			// calculate output power using gain and clip it to the limits
			var output = error * gain

				// Now limit rate of change of output (acceleration)
			if (!sign(error).equals(sign(lastOutput))){
				output = dV * sign(error) // If the output is changing direction, limit the rate of change
			} else if (absError > abs(lastOutput)) {
				if (output - lastOutput > dV) {
					output = lastOutput + dV // limit the rate of increase
				} else if (output - lastOutput < -dV) {
					output = lastOutput - dV // limit the rate of decrease
				}
			}
			output
		}
		lastOutput = output
		cycleTime.reset()
		return output
	}

	/**
	 * Leave everything else the same, Just restart the acceleration timer and set output to 0
	 */
	fun reset(everything: Boolean = false) {
		cycleTime.reset()
		inPosition = false
		if (everything) {
			lastOutput = 0.0
		}
	}
}