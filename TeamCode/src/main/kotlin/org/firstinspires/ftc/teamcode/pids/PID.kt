package org.firstinspires.ftc.teamcode.pids

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

open class PID(protected var kP: Double, protected val kI: Double, protected var kD: Double) {
    protected var totalError = 0.0
    protected var lastError = 0.0
    protected var lastTime = 0.0

    protected val _errors = mutableListOf<Double>()
    val errors: List<Double>
        get() = _errors

    open fun p(error: Double): Double {
        return kP * error
    }

    open fun i(error: Double): Double {
        if (abs(error) < 1) {
            totalError = 0.0
        }
        return kI * totalError
    }

    open fun d(currentError: Double, runtime: ElapsedTime): Double {
        return kD * (currentError - lastError) / (runtime.time() - lastTime)
    }

    open fun checkCorrection(error: Double) = abs(_errors.last()) < abs(error)

    open fun getCorrection(error: Double, runtime: ElapsedTime): Double {
        if (abs(error) < .001) {
            return 0.0
        }

        totalError += error
        lastError = error
        lastTime = runtime.time()

        return p(error) + i(error) + d(error, runtime).also {
            _errors.add(it)
        }
    }
}