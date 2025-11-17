package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(group = "!", name = "TeleOp - Red")
class TeleOp2Red : TeleOp2() {
    override val negateIfRed = -1
    override val negateIfBlue = +1
}