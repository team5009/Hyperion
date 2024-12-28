package ca.helios5009.hyperion.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

/**
 * HyperionMotor is a class that allows for the creation of motors for the robot to use.
 *
 * What makes this class different is it caches the power of the motor. This is to prevent
 * setting the power of the motor to the same value.
 * Every time the power is changed, the call from the control hub to the motor takes time.
 * Caching makes it so the motor doesn't have to be set to the same power.
 *
 * @param hardwareMap the hardware map to get the motor from.
 * @param motorName the name of the motor in the hardware map.
 *
 * @author Joshua
 * @see DcMotorEx
 */
class HyperionMotor(hardwareMap: HardwareMap, motorName: String) {
	val motor = hardwareMap[motorName] as DcMotorEx

	/**
	 * The power of the motor.
	 * This is cached to prevent setting the power to the same value.
	 */
	var power = 0.0
		set(value) {
			if (field != value) {
				field = value
				motor.power = power
			}
		}

	var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
		get() = motor.zeroPowerBehavior
		set(value) {
			motor.zeroPowerBehavior = value
		}

	/**
	 * Sets the mode of the motor.
	 * @param mode the mode to set.
	 */
	var mode get() = motor.mode
		set(value) {
			motor.mode = value
		}

	/**
	 * Returns the current reading of the encoder for this motor.
	 * The units for this reading, that is, the number of ticks per revolution, are specific to the motor/ encoder in question, and thus are not specified here.
	 * @return the current reading of the encoder for this motor.
	 */
	val position get() = motor.currentPosition

	/**
	 * Returns the current velocity of the motor, in ticks per second.
	 * @return the current velocity of the motor.
	 */
	val velocity get() = motor.velocity

	/**
	 * Returns whether the current consumption of this motor exceeds the alert threshold.
	 * @return true if the current consumption exceeds the alert threshold.
	 */
	val isOverCurrent get() = motor.isOverCurrent

	private var powerTolerance = 0.005

	init {
		motor.direction = DcMotorSimple.Direction.FORWARD
	}

	/**
	 * Sets the power of the motor with a tolerance.
	 * Simply better for performance as it uses a tolerance to set the power.
	 * @param power the power to set.
	 */
	fun setPowerWithTol(power: Double) {
		if (abs(power - this.power) >= powerTolerance || power == 0.0) {
			this.power = power
		}
	}

	/**
	 * Returns the current consumed by this motor.
	 * @param unit the unit of current to return.
	 */
	fun getCurrentDraw(unit: CurrentUnit): Double {
		return motor.getCurrent(unit)
	}

	/* Utility */

	/**
	 * Resets the encoder for this motor.
	 */
	fun resetEncoder(): HyperionMotor {
		motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
		motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
		return this
	}

	/**
	 * Reverse the motor.
	 */
	fun reverse(): HyperionMotor {
		motor.direction = DcMotorSimple.Direction.REVERSE
		return this
	}
	/**
	 * Sets the power differential that will be considered the same power.
	 * @param tolerance the power tolerance to set.
	 */
	fun setPowerTolerance(tolerance: Double): HyperionMotor {
		if (tolerance < 0) {
			throw IllegalArgumentException("Tolerance must be greater than 0")
		}
		powerTolerance = tolerance
		return this
	}

	/**
	 * Stops the motor.
	 */
	fun stop() {
		power = 0.0
	}
}