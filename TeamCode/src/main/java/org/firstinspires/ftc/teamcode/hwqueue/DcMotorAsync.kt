package org.firstinspires.ftc.teamcode.hwqueue

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.lang.ref.WeakReference

class DcMotorAsync(val around: DcMotor) {
    private val hw by LogicThread.hw

    private var setPowerTicket: WeakReference<HardwareQuery<Unit>>? = null

    var power: Double
        @JvmName("getPower") get() = querySync(hw) { around.power }
        @JvmName("setPower") set(power) {
            setPowerTicket?.get()?.revoke()
            val q = query {
                around.power = power
            }
            hw.supply(q)
            setPowerTicket = q.weakRef
        }

    private var directionTicket: WeakReference<HardwareQuery<Unit>>? = null

    var direction: DcMotorSimple.Direction
        @JvmName("getDirection") get() = querySync(hw) { around.direction }
        @JvmName("setDirection") set(direction) {
            directionTicket?.get()?.revoke()
            val q = query {
                around.direction = direction
            }
            hw.supply(q)
            directionTicket = q.weakRef
        }
}