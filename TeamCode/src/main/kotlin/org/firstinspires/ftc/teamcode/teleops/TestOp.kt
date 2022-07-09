package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.PIDS.LiftPID
import org.firstinspires.ftc.teamcode.components._angularOrientation
import org.firstinspires.ftc.teamcode.components.defaultBNO055IMUParameters
import org.firstinspires.ftc.teamcode.components.easytoggle.EasyToggles
import org.firstinspires.ftc.teamcode.components.logColorData
import org.firstinspires.ftc.teamcode.components.motors.Motors
import org.firstinspires.ftc.teamcode.components.motors.initializedMotors
import org.firstinspires.ftc.teamcode.components.motors.logMotorData
import org.firstinspires.ftc.teamcode.components.servos.Servos
import org.firstinspires.ftc.teamcode.components.servos.armsPositions
import org.firstinspires.ftc.teamcode.components.servos.initializedServos
import org.firstinspires.ftc.teamcode.util._get
import org.firstinspires.ftc.teamcode.util.initializableOnce
import kotlin.math.abs
import kotlin.math.absoluteValue

@TeleOp(name = "TestOpKt")
class TestOp : OpMode() {
    companion object { const val LIFT_TOP = 100 }

    private var motors: Motors by initializableOnce()
    private var servos: Servos by initializableOnce()

    private var imu: BNO055IMU by initializableOnce()
    private lateinit var angularOrientation: Orientation

    private var colorSensor: RevColorSensorV3 by initializableOnce()
    private var numElements = 0

    private val toggles = EasyToggles()

    val liftPid = LiftPID(.0075, .0, .002)
    var liftError = 0
    var liftTargetPos = 0

    override fun init() {
        motors = initializedMotors(hardwareMap)
        servos = initializedServos(hardwareMap)

        imu.initialize(defaultBNO055IMUParameters())
        angularOrientation = imu._angularOrientation()

        colorSensor = hardwareMap._get("color")
    }

    override fun loop() {
        updateToggles()

        drive()
        intake()
        duck()
        arm()
        macroLift()
        deposit()
        check()

        logData()
        telemetry.update()

        angularOrientation = imu._angularOrientation()
    }

    private fun updateToggles() = toggles.apply {
        up.state = gamepad1.dpad_up
        down.state = gamepad1.dpad_down
        _in.state = gamepad1.left_trigger > .5
        out.state = gamepad1.left_bumper
    }

    private fun drive() = with(gamepad1) {
        val triggered = abs(left_stick_y) > 0.1 || abs(left_stick_x) > 0.1 || abs(right_stick_x) > 0.1

        var flp = left_stick_y - left_stick_x - right_stick_x
        var frp = -left_stick_y - left_stick_x - right_stick_x
        var blp = left_stick_y + left_stick_x - right_stick_x
        var brp = -left_stick_y + left_stick_x - right_stick_x

        if (triggered) {
            val max = listOf(flp, frp, blp, brp).maxOf { abs(it) }.absoluteValue
            if (max > 1) {
                flp /= max
                frp /= max
                blp /= max
                brp /= max
            }
        }

        val powerMulti = if (triggered) {
            if (right_trigger > 0.5) 0.35 else 1.0
        } else 0.0

        motors.frontLeft.power = flp * powerMulti
        motors.frontRight.power = frp * powerMulti
        motors.backLeft.power = blp * powerMulti
        motors.backRight.power = brp * powerMulti

        if (powerMulti == .35) {
            motors.logMotorData(telemetry) { it.power }
        }
    }

    private fun intake() = with(servos) {
        if (toggles._in.nowTrue() || toggles.out.nowTrue()) {
            fold.position = .27
            armsPositions = .2

            if (toggles._in.nowTrue()) {
                motors.intake.power = 1.0
            } else {
                motors.intake.power = -1.0
            }
        } else if (toggles.out.nowFalse() || toggles._in.nowFalse()) {
            motors.intake.power = 0.0
            fold.position = .5
            armsPositions = .5
        }

        if (numElements != 0 && gamepad1.left_trigger > .5) {
            motors.intake.power = -1.0
        }
    }

    private fun deposit() = when {
        gamepad1.right_trigger > .5 -> servos.dep.position = .6
        numElements == 2 -> servos.dep.position = .45
        numElements == 1 -> servos.dep.position = .4
        numElements == 0 -> servos.dep.position = .5
        else -> Unit
    }

    private fun check() = when {
        colorSensor.alpha() > 7500 -> numElements = 2
        colorSensor.alpha() > 1000 -> numElements = 1
        else -> numElements = 0
    }

    private fun duck() = when {
        gamepad1.right_bumper && gamepad1.a -> motors.duck.power = 1.0
        gamepad1.right_bumper -> motors.duck.power = 0.5
        else -> motors.duck.power = 0.0
    }

    private fun arm() = if (gamepad1.x) {
        servos.armsPositions = .5
    } else if (gamepad1.y) {
        servos.armsPositions = .83
    } else { Unit }

    private fun macroLift() {
        liftError = liftTargetPos - motors.lift.currentPosition

        motors.lift.power = Range.clip(liftPid.getCorrection(liftError.toDouble()), -.7, 1.0)

        if (!toggles.down.nowTrue()) return

        if (servos.arm1.position > .4) {
            liftTargetPos = LIFT_TOP
            servos.armsPositions = .83
        } else {
            liftTargetPos = 0
            servos.armsPositions = .5
        }
    }

    private fun logData() {
        colorSensor.logColorData(telemetry)
        telemetry.addData("Lift motor pos", motors.lift.currentPosition)
        telemetry.addData("Lift motor power", motors.lift.power)
        telemetry.addData("Lift target pos", liftTargetPos)
    }
}