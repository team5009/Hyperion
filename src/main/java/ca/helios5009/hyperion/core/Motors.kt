package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.hardware.HyperionMotor
import com.qualcomm.robotcore.hardware.DcMotor
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
	var mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
		set(value) {
			field = value
			motorList.forEach { it.mode = value }
		}
	var zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
		set(value) {
			field = value
			motorList.forEach { it.zeroPowerBehavior = value }
		}

	init {
		mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
		zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
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
		val highestPower = abs(drive) + abs(strafe) + abs(rotate)
		val powerRatioCalc = maxOf(highestPower, 1.0)

		listOf(
			drive + strafe + rotate,
			drive - strafe - rotate,
			drive - strafe + rotate,
			drive + strafe - rotate
		).forEachIndexed { index, power ->
			motorList[index].setPowerWithTol(power / powerRatioCalc * powerRatio.get()) // To apply the power ratio
		}
	}

	/**
	 * Set the power ratio of the motors.
	 * @param ratio The ratio of the power of the motors.
	 */
	fun setPowerRatio(ratio: Double) {
		if (ratio <= 0 || ratio > 1) throw IllegalArgumentException("Max Power must be between 0 and 1")
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
		val highestPower = abs(drive) + abs(strafe) + abs(rotate)
		val powerRatioCalc = maxOf(highestPower, 1.0)

		listOf(
			drive + strafe + rotate,
			drive - strafe - rotate,
			drive - strafe + rotate,
			drive + strafe - rotate
		).forEachIndexed { index, power ->
			motorList[index].power = power / powerRatioCalc * powerRatio.get() // To apply the power ratio
		}
	}

	fun stop() {
		fl.stop()
		fr.stop()
		bl.stop()
		br.stop()
	}
}