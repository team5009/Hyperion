package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.hardware.HyperionMotor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs

/**
 * Motor Class to handle all the Motor movements. This class handles your drive train.
 * Acts as a wrapper for the motors of any FTC robot using mecanum wheels.
 * It does
 */
class Motors(
	hardwareMap: HardwareMap,
	frontLeft: String,
	frontRight: String,
	backLeft: String,
	backRight: String
) {
	private val powerRatio = AtomicReference(1.0) // The max power of the motors
	private val fl: HyperionMotor = HyperionMotor(hardwareMap, frontLeft).reverse()
	private val fr: HyperionMotor = HyperionMotor(hardwareMap, frontRight)
	private val bl: HyperionMotor = HyperionMotor(hardwareMap, backLeft).reverse()
	private val br: HyperionMotor = HyperionMotor(hardwareMap, backRight)
	private val motorList = arrayOf(fl, fr, bl, br)

	init {
		motorList.forEach {
			it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
			it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
		}
	}

	/**
	 * Given the drive, strafe, and rotate values, move the robot.
	 * Don't use this method if you are using a gamepad.
	 * @param drive The forward/backward power of the robot.
	 * @param strafe The left/right power of the robot.
	 * @param rotate The rotation power of the robot.
	 *
	 * @see gamepadMove for gamepad controls.
	 */
	fun move(drive: Double, strafe: Double, rotate: Double) {
		val maxPower = abs(drive) + abs(strafe) + abs(rotate)
		val powRatio = powerRatio.get()
		val max = minOf(powRatio, maxPower)
		val powerList = doubleArrayOf(
			drive + strafe + rotate,    // The power of the front left motor
			drive - strafe - rotate,    // The power of the front right motor
			drive - strafe + rotate,    // The power of the back left motor
			drive + strafe - rotate     // The power of the back right motor
		)

		motorList.forEachIndexed { index, motor ->
			motor.setPower(powerList[index] / max * powRatio)
		}
	}

	/**
	 * Set the power ratio of the motors.
	 * @param ratio The ratio of the power of the motors.
	 */
	fun setPowerRatio(ratio: Double) {
		if (ratio <= 0 || ratio > 1) throw IllegalArgumentException("Power ratio must be between 0 and 1")
		powerRatio.set(ratio)
	}

	/**
	 * Given a gamepad, move the robot.
	 * Don't use this method if you are not using a gamepad.
	 * @param drive The forward/backward power of the robot.
	 * @param strafe The left/right power of the robot.
	 * @param rotate The rotation power of the robot.
	 * @see move for non-gamepad controls.
	 */
	fun gamepadMove(drive: Double, strafe: Double, rotate: Double) {
		val maxPower = abs(drive) + abs(strafe) + abs(rotate)
		val powRatio = powerRatio.get()
		val max = minOf(powRatio, maxPower)

		val powerList = doubleArrayOf(
			drive + strafe + rotate,
			drive - strafe - rotate,
			drive - strafe + rotate,
			drive + strafe - rotate
		)

		motorList.forEachIndexed { index, motor ->
			motor.power = powerList[index] / max * powRatio
		}
	}

	fun stop () {
		fl.stop()
		fr.stop()
		bl.stop()
		br.stop()
	}
}