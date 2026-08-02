package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry

class GamepadState (private val gp1: Gamepad, private val gp2: Gamepad) {

    var rb1 = false
    var wasrb1 = false

    var lb1 = false
    var waslb1 = false

    var y2 = false
    var wasy2 = false

    var x2 = false
    var wasx2 = false

    var b1 = false
    var wasb1 = false

    var jx1 = 0.0
    var jy1 = 0.0
    var rx1 = 0.0

    fun update(){
//        if (gp1 == null){
//            telemetry.addData("init gp1","")
//            return
//        }
//        if (gp2 == null){
//            telemetry.addData("init gp2","")
//            return
//        }
//        gp1 = gp1!!
//        gp2 = gp2!!

        //field centric drive (DM)
        jx1 = gp1.left_stick_x.toDouble()
        jy1 = -gp1.left_stick_y.toDouble()
        rx1 = gp1.right_stick_x.toDouble()

        //intake on (ISM)
        rb1 = gp1.right_bumper && !wasrb1
        wasrb1 = gp1.right_bumper
        //intake off (ISM)
        lb1 = gp1.left_bumper && !waslb1
        waslb1 = gp1.left_bumper

        //shoot (ISM)
        y2 = gp2.y && !wasy2
        wasy2 = gp2.y

        //cycle shooting mode (AM)
        x2 = gp2.x && !wasx2
        wasx2 = gp2.x

        //hybrid intake
        b1 = gp1.b && !wasb1
        wasb1 = gp1.b

    }
}