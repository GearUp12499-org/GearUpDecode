package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.systems.REmover.RobotPose

class PoseSet private constructor(val invert: Boolean) {
    private val RobotPose.bind: RobotPose
        get() = if (invert) RobotPose(
            this.x,
            -this.y,
            -this.a
        ) else this

    companion object {
        private val farStart = RobotPose(-63.5, -16.0, Math.PI)
        private val goalStart = RobotPose(54.33, -51.34, 2.19)
        private val readAT = RobotPose(46.92, -11.92, -2.55)
        private val closeShoot = RobotPose(60.0, -10.48, 0.55 * Math.PI)
        private val midShoot = RobotPose(12.5, -12.0, 3 * Math.PI / 4)
        private val farShoot = RobotPose(-55.0, -12.39, 2.76)
        private val set1pos = RobotPose(12.0, -30.375, -Math.PI / 2)
        private val set1out = RobotPose(12.0, -55.625, -Math.PI / 2)
        private val set2pos = RobotPose(-12.0, -30.375, -Math.PI / 2)
        private val set2out = RobotPose(-12.0, -63.125, -Math.PI / 2)
        private val set3pos = RobotPose(-36.0, -30.375, -Math.PI / 2)
        private val set3out = RobotPose(-36.0,  -63.125, -Math.PI / 2)
        private val blueBase = RobotPose(-38.0, -33.0, 0.0)
        private val gateWaypoint = RobotPose(0.0, -48.0, 0.0)
        private val gatePos = RobotPose(0.0, -55.0, 0.0)
        private val shootTarget = RobotPose(72.0 - 6.0, -(72.0 - 6.0), 0.0)
        private val shootMeasure = RobotPose(57.0, -57.0, 0.0)

        @JvmField
        val RED = PoseSet(false)

        @JvmField
        val BLUE = PoseSet(true)
    }

    @JvmField
    val farStart = Companion.farStart.bind
    @JvmField
    val goalStart = Companion.goalStart.bind
    @JvmField
    val readAT = Companion.readAT.bind
    @JvmField
    val closeShoot = Companion.closeShoot.bind
    @JvmField
    val midShoot = Companion.midShoot.bind
    @JvmField
    val farShoot = Companion.farShoot.bind
    @JvmField
    val set1pos = Companion.set1pos.bind
    @JvmField
    val set1out = Companion.set1out.bind
    @JvmField
    val set2pos = Companion.set2pos.bind
    @JvmField
    val set2out = Companion.set2out.bind
    @JvmField
    val set3pos = Companion.set3pos.bind
    @JvmField
    val set3out = Companion.set3out.bind
    @JvmField
    val blueBase = Companion.blueBase.bind
    @JvmField
    val gateWaypoint = Companion.gateWaypoint.bind
    @JvmField
    val gatePos = Companion.gatePos.bind
    @JvmField
    val shootTarget = Companion.shootTarget.bind
    @JvmField
    val shootMeasure = Companion.shootMeasure.bind
}