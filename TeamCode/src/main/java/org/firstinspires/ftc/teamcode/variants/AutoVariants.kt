@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.variants

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Auto1
import org.firstinspires.ftc.teamcode.Auto1B
import org.firstinspires.ftc.teamcode.Auto2
import org.firstinspires.ftc.teamcode.Auto2B
import org.firstinspires.ftc.teamcode.StateAuto

//import org.firstinspires.ftc.teamcode.Auto2
//import org.firstinspires.ftc.teamcode.Auto2B
//import org.firstinspires.ftc.teamcode.Auto3

@Autonomous
class Auto1Red : Auto1(true)

@Autonomous
class Auto1Blue : Auto1(false)

@Autonomous
class Auto1BRed : Auto1B(true)

@Autonomous
class Auto1BBlue : Auto1B(false)

@Autonomous
class Auto2Red : Auto2(true)

@Autonomous
class Auto2Blue : Auto2(false)

@Autonomous
class Auto2BRed : Auto2B(true)

@Autonomous
class Auto2BBlue : Auto2B(false)

@Autonomous
class StateAutoRed: StateAuto(true)

@Autonomous
class StateAutoBlue: StateAuto(false)

//@Autonomous
//class Auto3Red : Auto3(true)
//
//@Autonomous
//class Auto3Blue : Auto3(false)