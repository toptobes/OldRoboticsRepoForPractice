package org.firstinspires.ftc.teamcode.components.servos

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util._get

typealias SDirection = Servo.Direction

@JvmOverloads
fun initializedServo(name: String, hwMap: HardwareMap, pos: Double, reversed: Boolean = false): Servo {
    return hwMap
        ._get<Servo>(name)
        .apply {
            position = pos
            direction = if (reversed) SDirection.REVERSE else SDirection.FORWARD
        }
}

var Servos.armsPositions: Double
    get() = arm1.position
    set(value) {
        arm1.position = value
        arm2.position = value
    }

fun Servos.logMotorData(telemetry: Telemetry, dataSupplier: (Servo) -> Any) {
    telemetry.addData("arm1", dataSupplier(arm1))
    telemetry.addData("arm2", dataSupplier(arm2))
    telemetry.addData("dep", dataSupplier(dep))
    telemetry.addData("fold", dataSupplier(fold))
    telemetry.addData("cap", dataSupplier(cap))
}