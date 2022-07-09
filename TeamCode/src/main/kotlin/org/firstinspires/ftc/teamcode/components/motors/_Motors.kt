package org.firstinspires.ftc.teamcode.components.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util._get

typealias ZPB = DcMotor.ZeroPowerBehavior
typealias DCDirection = DcMotorSimple.Direction

@JvmOverloads
fun initializedMotor(name: String, hwMap: HardwareMap, zpb: ZPB = ZPB.BRAKE, reversed: Boolean = false): DcMotorEx {
    return hwMap
        ._get<DcMotorEx>(name)
        .apply {
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = zpb
            direction = if (reversed) DCDirection.REVERSE else DCDirection.REVERSE
        }
}

fun initializedMotors(hardwareMap: HardwareMap) = Motors().apply {
    frontLeft = initializedMotor("FL", hardwareMap)
    frontRight = initializedMotor("FR", hardwareMap)
    backLeft = initializedMotor("BL", hardwareMap)
    backRight = initializedMotor("BR", hardwareMap)

    intake = initializedMotor("IN", hardwareMap, zpb = ZPB.FLOAT)
    duck = initializedMotor("DU", hardwareMap, zpb = ZPB.FLOAT)

    lift = initializedMotor("LI", hardwareMap, reversed = true)
}

fun Motors.logMotorData(telemetry: Telemetry, dataSupplier: (DcMotorEx) -> Any) {
    telemetry.addData("Front-left power:", dataSupplier(frontLeft))
    telemetry.addData("Front-right power:", dataSupplier(frontRight))
    telemetry.addData("Back-left power:", dataSupplier(backLeft))
    telemetry.addData("Back-right power:", dataSupplier(backRight))
}