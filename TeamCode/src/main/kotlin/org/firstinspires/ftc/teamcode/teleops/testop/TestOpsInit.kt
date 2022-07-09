package org.firstinspires.ftc.teamcode.teleops.testop

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.components.motors.Motors
import org.firstinspires.ftc.teamcode.components.motors.initializedMotor
import org.firstinspires.ftc.teamcode.components.servos.Servos
import org.firstinspires.ftc.teamcode.components.servos.initializedServo

typealias ZPB = DcMotor.ZeroPowerBehavior

fun initializedMotors(hardwareMap: HardwareMap) = Motors().apply {
    frontLeft = initializedMotor("FL", hardwareMap)
    frontRight = initializedMotor("FR", hardwareMap)
    backLeft = initializedMotor("BL", hardwareMap)
    backRight = initializedMotor("BR", hardwareMap)

    intake = initializedMotor("IN", hardwareMap, zpb = ZPB.FLOAT)
    duck = initializedMotor("DU", hardwareMap, zpb = ZPB.FLOAT)

    lift = initializedMotor("LI", hardwareMap, reversed = true)
}

fun initializedServos(hardwareMap: HardwareMap) = Servos().apply {
    arm1 = initializedServo("arm1", hardwareMap, pos = .5, reversed = true)
    arm2 = initializedServo("arm2", hardwareMap, pos = .5)
    dep = initializedServo("dep", hardwareMap, pos = .57)
    fold = initializedServo("fold", hardwareMap, pos = .5)
    cap = initializedServo("cap", hardwareMap, pos = .6)
}

fun defaultBNO055IMUParameters() = BNO055IMU.Parameters().apply {
    angleUnit = BNO055IMU.AngleUnit.DEGREES
    accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
    calibrationDataFile = "BNO055IMUCalibration.json"
    loggingEnabled = true
    loggingTag = "IMU"
}
