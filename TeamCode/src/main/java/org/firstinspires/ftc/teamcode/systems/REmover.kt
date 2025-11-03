package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.Task
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

class REmover(private val hardware: CompBotHardware) : Task<REmover>() {

    companion object {
        const val kp = 0.2
        const val kd = 43.75
    }

    override fun onTick(): Boolean {
        return false
    }

    init {
        require(CompBotHardware.Locks.DRIVE_MOTORS)
    }

    fun drive2Pose(xya: DoubleArray): Task<*> {
        val tgtx = xya[0]
        val tgty = xya[1]
        val tgta = xya[2]


        return object : Anonymous() {

            lateinit var timeout: ElapsedTime
            lateinit var runtime: ElapsedTime
            var deltaTime = 0.0
            var currenTime = 0.0
            var prevTime = 0.0
            var prevDeltaAll = 0.0

            override fun onStart() {
                timeout = ElapsedTime(ElapsedTime.Resolution.SECONDS)
                runtime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
                currenTime = runtime.time()
                prevTime = runtime.time()
            }

            override fun onTick(): Boolean {
                currenTime = runtime.time()

                val Timeout = timeout.time()

                hardware.pinpoint.update()

                val yVelocity = hardware.pinpoint.getVelY(DistanceUnit.INCH)
                val xVelocity = hardware.pinpoint.getVelX(DistanceUnit.INCH)

                val speed = hypot(xVelocity, yVelocity)
                val currentPose = hardware.pinpoint.position

                val currentx = currentPose.getX(DistanceUnit.INCH)
                val currenty = currentPose.getY(DistanceUnit.INCH)
                val currentTheta = currentPose.getHeading(AngleUnit.RADIANS)

                val deltax = tgtx - currentx
                val deltay = tgty - currenty
                var deltaA = tgta - currentTheta
                deltaA = deltaA % (2 * (Math.PI))
                if (deltaA > Math.PI) {
                    deltaA -= 2 * Math.PI
                }

                if (abs(deltax) < 0.5 && abs(deltay) < 0.5 && abs(deltaA) < Math.PI / 24 && speed < 10 || Timeout > 1) {

                    hardware.frontLeft.power = 0.0
                    hardware.frontRight.power = 0.0
                    hardware.backLeft.power = 0.0
                    hardware.backRight.power = 0.0
                    return true

                }

                val R = 9.375
                val F = cos(currentTheta) * deltax + sin(currentTheta) * deltay
                val S = sin(currentTheta) * deltax - cos(currentTheta) * deltay
                val W = R * deltaA
                val deltaAll = sqrt((F * F) + (S * S) + (W * W))

                if (abs(deltaAll - prevDeltaAll) > 0.5) {
                    timeout.reset()
                }

                var DFL = F + S - W
                var DBL = F - S - W
                var DFR = F - S + W
                var DBR = F + S + W


                //rescale the four speeds so the largest is +/- 1
                val tempMax1 = max(abs(DFL), abs(DBL))
                val tempMax2 = max(abs(DFR), abs(DBR))
                var scale = max(tempMax1, tempMax2)

                if (scale < 0.01) {
                    scale = 0.01
                }

                DFL /= scale
                DBL /= scale
                DFR /= scale
                DBR /= scale

                deltaTime = max(currenTime - prevTime, 0.001)

                val pid = kp * deltaAll + kd * (deltaAll - prevDeltaAll) / deltaTime

                if (abs(pid) < 1.0) {
                    DFL *= pid
                    DBL *= pid
                    DFR *= pid
                    DBR *= pid
                }

                val PFL = speed2Power(DFL)
                val PFR = speed2Power(DFR)
                val PBL = speed2Power(DBL)
                val PBR = speed2Power(DBR)

                hardware.frontLeft.power = PFL
                hardware.backLeft.power = PBL
                hardware.frontRight.power = PFR
                hardware.backRight.power = PBR

                return false
            }
        }
    }

    fun speed2Power(speed: Double): Double {
        val threshold = 0.2

        if (abs(speed) < 0.001) {
            return 0.0
        }

        if (speed > 0) {
            return threshold + ((1 - threshold) * speed)
        }
        if (speed < 0) {
            return -threshold + ((1 - threshold) * speed)
        }

        throw IllegalArgumentException()
    }
}