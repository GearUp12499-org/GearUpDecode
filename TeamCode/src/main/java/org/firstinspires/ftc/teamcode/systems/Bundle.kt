package org.firstinspires.ftc.teamcode.systems

import org.firstinspires.ftc.teamcode.hardware.CompBotHardware

data class Bundle(
    val aprilTag: AprilTag,
    val indexer: Indexer,
    val shooter: Shooter,
    val hw: CompBotHardware?
)