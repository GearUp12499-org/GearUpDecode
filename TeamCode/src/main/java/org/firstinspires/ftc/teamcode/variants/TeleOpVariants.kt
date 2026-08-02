@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.variants

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeleOp3

@TeleOp
class TeleOpRed : TeleOp3(true)

@TeleOp
class TeleOpBlue : TeleOp3(false)

@TeleOp
class StateTeleOpRed: StateTeleop(true)

@TeleOp
class StateTeleopBlue: StateTeleop(false)