package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.REmover.RobotPose
import kotlin.math.PI

class PoseSet private constructor(val invert: Boolean) {
    private val RobotPose.bind: RobotPose
        get() = if (invert) RobotPose(
            this.x,
            -this.y,
            -this.a,
            this.turret?.let { -it }
        ) else this

    // ENTER RED VALUES (usually +X -Y) ONLY!
    // Poses will automatically be mirrored for Blue OpModes
    companion object {
        private val farStart = RobotPose(-64.75, -17.25, Math.PI / 2)

        //        private val goalStart = RobotPose(49.64, -54.48, -2.318)
//        private val goalStart = RobotPose(49.64, -54.48, -2.5248)
//        private val goalStart = RobotPose(49.64, -54.48, -2.514076654)
//        private val goalStart = RobotPose(55.05, -48.61, Math.PI / 2)
        private val goalStart = RobotPose(64.75, -30.25, Math.PI / 2)

        // FIXME: onshape this

        //58, -56
        private val goalAT = RobotPose(66.0, -66.0, -2.318)
        private val goalAtFAR = RobotPose (58.0, -56.0, -2.318)
        private val closeShoot = RobotPose(60.0, -12.48, 0.55 * Math.PI)
        private val closeShoot2 = RobotPose(36.0, -36.0, 2.39)
        private val closeShoot3 = RobotPose(48.66, -25.0, 2.06)
        private val midShoot = RobotPose(24.0, -24.0, 2.39, -77.0)
        private val midShoot2 = RobotPose(36.0, -12.0, 2 * Math.PI / 3, -93.0)
        private val farShoot = RobotPose(-55.0, -12.39, 2.705 )
        private val farShoot2 = RobotPose(-64.0, -29.0, -Math.PI/2,-106.0)


        //private val set1pos = RobotPose(12.0, -24.0, -Math.PI / 2)
        private val set1pos = RobotPose(12.0, -18.0, -Math.PI / 2)
        private val set1out = RobotPose(12.0, -54.625, -Math.PI / 2)

        private val set1curve =  RobotPose(12.0,-20.0,0.0)
        private val set2pos = RobotPose(-12.0, -24.0, -Math.PI / 2)
        private val set2out = RobotPose(-12.0, -62.625, -Math.PI / 2)

        private val set2curve = RobotPose(-12.6,-5.0,0.0)
        private val set2exit = RobotPose(0.0, -32.0, -Math.PI)
        private val set3pos = RobotPose(-36.0, -24.0, -Math.PI / 2)
        private val set3out = RobotPose(-36.0, -62.625, -Math.PI / 2)

        private val set3curve = RobotPose(-36.6,2.0,0.0)
        private val set4pos = RobotPose(-57.5, -61.0, -Math.PI / 2)
        private val set4out = RobotPose(-64.5, -60.0, -Math.PI / 2)
        private val overflowPos1 = RobotPose(-64.75, -64.6, -Math.PI / 2)

        private val overflowPos3 = RobotPose(-64.75, -30.0, -Math.PI / 2)
        private val overflowPos2 = RobotPose(-60.0, -65.6, 0.0)

        private val set4out2 = RobotPose(-64.5, -61.0, -Math.PI / 2)
        private val auto2park = RobotPose(-42.04, -18.498, 2.76)
        private val auto1finish = RobotPose(0.0, -48.0, Math.PI)
        private val blueBase = RobotPose(-38.0, -33.0, 0.0)
        private val gateWaypoint = RobotPose(0.0, -48.0, 0.0)
        private val gatePos = RobotPose(3.0, -65.0, 0.0)
        private val gatePos2 = RobotPose(3.0, -53.0, 0.0)
        private val gatePos3 = RobotPose(3.0, -49.0, 0.0)

        private val gobble0 = RobotPose(-3.42, -24.0, -PI / 2)
        private val gobble1 = RobotPose(-3.42, -58.0, -PI / 2)
        private val gobble1b = RobotPose(-15.0, -52.0, -PI / 2)
        private val gobble1c = RobotPose(-15.0, -64.5, -5 * PI / 12)

        private val gobble2 = RobotPose(-17.11, -66.08, -0.023)

        private val gobble3 = RobotPose(-4.5, -62.0, -1.5732)

        private val gobbleSet = RobotPose(-4.5, -24.0, -Math.PI / 2)

        private val gobble4 = RobotPose(-9.14, -62.44,-1.0614)
        private val gobble5 = RobotPose(-10.42, -60.36,-PI/2)

        private val gobble6 = RobotPose(-20.03, -65.52, -0.785)
        private val gobble7 = RobotPose(-10.0, -65.52, 0.0)

        private val gobbleCurve = RobotPose(-20.0,-20.0, 0.0)
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
    val goalAT = Companion.goalAT.bind

    @JvmField
    val goalAtFAR = Companion.goalAtFAR.bind

    @JvmField
    val closeShoot = Companion.closeShoot.bind

    @JvmField
    val closeShoot2 = Companion.closeShoot2.bind

    @JvmField
    val closeShoot3 = Companion.closeShoot3.bind

    @JvmField
    val midShoot = Companion.midShoot.bind

    @JvmField
    val midShoot2 = Companion.midShoot2.bind

    @JvmField
    val farShoot = Companion.farShoot.bind

    @JvmField
    val farShoot2 = Companion.farShoot2.bind


    @JvmField
    val set1pos = Companion.set1pos.bind

    @JvmField
    val set1out = Companion.set1out.bind

    @JvmField
    val set1curve = Companion.set1curve.bind

    @JvmField
    val set2pos = Companion.set2pos.bind

    @JvmField
    val set2out = Companion.set2out.bind

    @JvmField
    val set2curve = Companion.set2curve.bind

    @JvmField
    val set2exit = Companion.set2exit.bind

    @JvmField
    val set3pos = Companion.set3pos.bind

    @JvmField
    val set3out = Companion.set3out.bind

    @JvmField
    val set3curve = Companion.set3curve.bind

    @JvmField
    val set4pos = Companion.set4pos.bind

    @JvmField
    val gobbleSet = Companion.gobbleSet.bind

    @JvmField
    val set4out = Companion.set4out.bind

    @JvmField
    val overflowPos1 = Companion.overflowPos1.bind

    @JvmField
    val overflowPos2 = Companion.overflowPos2.bind

    @JvmField
    val overflowPos3 = Companion.overflowPos3.bind

    @JvmField
    val auto2park = Companion.auto2park.bind

    @JvmField
    val auto1finish = Companion.auto1finish.bind

    @JvmField
    val blueBase = Companion.blueBase.bind

    @JvmField
    val gateWaypoint = Companion.gateWaypoint.bind

    @JvmField
    val gatePos = Companion.gatePos.bind

    @JvmField
    val gatePos2 = Companion.gatePos2.bind

    @JvmField
    val gatePos3 = Companion.gatePos3.bind

    @JvmField
    val gobble0 = Companion.gobble0.bind

    @JvmField
    val gobble1 = Companion.gobble1.bind

    @JvmField
    val gobble1b = Companion.gobble1b.bind

    @JvmField
    val gobble1c = Companion.gobble1c.bind

    @JvmField
    val gobble2 = Companion.gobble2.bind

    @JvmField
    val gobble3 = Companion.gobble3.bind

    @JvmField
    val gobble4 = Companion.gobble4.bind

    @JvmField
    val gobble5 = Companion.gobble5.bind

    @JvmField
    val gobble6 = Companion.gobble6.bind

    @JvmField
    val gobbleCurve = Companion.gobbleCurve.bind

    @JvmField
    val gobble7 = Companion.gobble7.bind

    @JvmField
    val shootTarget = Companion.shootTarget.bind

    @JvmField
    val shootMeasure = Companion.shootMeasure.bind
}