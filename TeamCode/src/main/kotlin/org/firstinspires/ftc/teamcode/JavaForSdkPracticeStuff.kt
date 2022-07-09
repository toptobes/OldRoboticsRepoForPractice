@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit as DU
import org.firstinspires.ftc.teamcode.util._get

lateinit var hwMap: HardwareMap

var DcMotorSimple._power: Number
    get() = throw IllegalStateException("Use the 'power' property to read instead")
    set(value) {
        power = value.toDouble()
    }

object ProgrammingBoard1 {
    val colorSensor: ColorSensor = hwMap._get("sensor_color_distance")
    fun getAmountBlue() = colorSensor.blue()
}

object ProgrammingBoard2 {
    val distanceSensor: DistanceSensor = hwMap._get("sensor_color_distance")
    val motor = hwMap._get<DcMotorEx>("motor")
    tailrec fun imaginaryLoop() {
        motor._power = if (distanceSensor.getDistance(DU.CM) < 10) 0 else .5
        imaginaryLoop()
    }
}