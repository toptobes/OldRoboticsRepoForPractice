@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.components.motors._power
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit as DU
import org.firstinspires.ftc.teamcode.util._get

lateinit var hwMap: HardwareMap

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