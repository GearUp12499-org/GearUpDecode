package org.firstinspires.ftc.teamcode


class Wait {

    public var startTime = System.nanoTime()

    fun waitSec(waitSeconds: Double): Boolean {
        val now = System.nanoTime().toDouble()
        if ((now - startTime)/1e9 > waitSeconds){
            return true
        } else {
            return false
        }
    }

//    fun waitForContinuous(waitSeconds: Long, timeout: Long = 1L): Boolean{
//
//    }
}