package org.firstinspires.ftc.teamcode.components

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation

fun defaultBNO055IMUParameters() = BNO055IMU.Parameters().apply {
    angleUnit = BNO055IMU.AngleUnit.DEGREES
    accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
    calibrationDataFile = "BNO055IMUCalibration.json"
    loggingEnabled = true
    loggingTag = "IMU"
}

@JvmOverloads
fun BNO055IMU._angularOrientation(
    axesReference: AxesReference = AxesReference.INTRINSIC,
    axesOrder: AxesOrder = AxesOrder.ZYX,
    angleUnit: AngleUnit = AngleUnit.DEGREES
): Orientation {
    return getAngularOrientation(axesReference, axesOrder, angleUnit)
}
