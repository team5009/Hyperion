package ca.helios5009.hyperion.core

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs

/**
 * Motor Class to handle all the Motor movements
 */
class Motors(
	hardwareMap: HardwareMap,
	frontLeft: String,
	frontRight: String,
	backLeft: String,
	backRight: String
) {
	val powerRatio = AtomicReference(1.0)
	private val fl: HyperionMotor = HyperionMotor(hardwareMap.get(frontLeft) as DcMotorEx)
	private val fr: HyperionMotor = HyperionMotor(hardwareMap.get(frontRight) as DcMotorEx)
	private val br: HyperionMotor = HyperionMotor(hardwareMap.get(backLeft) as DcMotorEx)
	private val bl: HyperionMotor = HyperionMotor(hardwareMap.get(backRight) as DcMotorEx)
	init {
		val motorList = arrayOf(fl, fr, br, bl)
		motorList[0].motor.direction = DcMotorSimple.Direction.REVERSE
		motorList[2].motor.direction = DcMotorSimple.Direction.REVERSE
		motorList.forEach {
			it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
			it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
		}
	}

	fun move(drive: Double, strafe: Double, rotate: Double) {
		val maxPower = abs(drive) + abs(strafe) + abs(rotate)
		val max = if (maxPower < 0.15) 0.15 else maxOf(1.0, maxPower)/powerRatio.get() // 0.8 is the max power of the motors, if the number is greater than 0.8, it will be divided by 0.8

		fl.setPower((drive + strafe + rotate) / max)
		fr.setPower((drive - strafe - rotate) / max)
		bl.setPower((drive - strafe + rotate) / max)
		br.setPower((drive + strafe - rotate) / max)
	}

	fun stop () {
		fl.stop()
		fr.stop()
		bl.stop()
		br.stop()
	}
}