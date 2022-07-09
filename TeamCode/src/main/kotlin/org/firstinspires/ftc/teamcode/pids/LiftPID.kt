package org.firstinspires.ftc.teamcode.pids

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

//TODO: Rethink the inheritance
class LiftPID(kP: Double, kI: Double, kD: Double) : PID(kP, kI, kD) {
    fun d(currentError: Double): Double {
        return kD * (currentError - lastError)
    }

    fun setPD(kP: Double, kD: Double) {
        super.kP = kP
        super.kD = kD
    }

    fun getCorrection(error: Double): Double {
        if (abs(error) < .001) {
            return 0.0
        }

        totalError += error
        lastError = error

        return p(error) + i(error) + d(error).also {
            _errors.add(it)
        }
    }

    override fun d(currentError: Double, runtime: ElapsedTime) = d(currentError)
    override fun getCorrection(error: Double, runtime: ElapsedTime) = getCorrection(error)
}