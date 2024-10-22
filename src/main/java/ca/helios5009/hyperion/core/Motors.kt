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
	private val fl: HyperionMotor = HyperionMotor(hardwareMap, frontLeft)
	private val fr: HyperionMotor = HyperionMotor(hardwareMap, frontRight)
	private val bl: HyperionMotor = HyperionMotor(hardwareMap, backLeft)
	private val br: HyperionMotor = HyperionMotor(hardwareMap, backRight)
	init {
		val motorList = arrayOf(fl, fr, bl, br)
		motorList[0].motor.direction = DcMotorSimple.Direction.REVERSE
		motorList[2].motor.direction = DcMotorSimple.Direction.REVERSE
		motorList.forEach {
			it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
			it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
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
		val max = maxOf(powerRatio.get(), maxPower)

		fl.setPower((drive + strafe + rotate) / max)
		fr.setPower((drive - strafe - rotate) / max)
		bl.setPower((drive - strafe + rotate) / max)
		br.setPower((drive + strafe - rotate) / max)
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
	 * @param gamepad The gamepad to get the values from.
	 * @see move for non-gamepad controls.
	 */
	fun gamepadMove(drive: Double, strafe: Double, rotate: Double) {
		val maxPower = abs(drive) + abs(strafe) + abs(rotate)
		val max = maxOf(powerRatio.get(), maxPower)

		fl.setPowerWithoutTolerance((drive + strafe + rotate) / max)
		fr.setPowerWithoutTolerance((drive - strafe - rotate) / max)
		bl.setPowerWithoutTolerance((drive - strafe + rotate) / max)
		br.setPowerWithoutTolerance((drive + strafe - rotate) / max)
	}

	fun stop () {
		fl.stop()
		fr.stop()
		bl.stop()
		br.stop()
	}
}