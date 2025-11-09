package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver

@TeleOp
class HardwareTestCompBot : HardwareTestBase() {
    var headingMessage: String? = null
    lateinit var hardware: CompBotHardware

    fun buildTelemetry() {
        telemetry.addLine("press START to quit early")
        telemetry.addLine()
        headingMessage?.split("\n")?.forEach(telemetry::addLine)
        buildReport().split("\n").forEach(telemetry::addLine)

        telemetry.update()
    }

    private fun testPinpointSetup() = object: TestableAction("pinpoint") {
        override fun onStart() {
            super.onStart()
            val p = hardware.pinpoint
            p.resetPosAndIMU()
        }

        override fun testIt() {
            val p = hardware.pinpoint
            p.update()
            reason = "${p.deviceStatus}"
            if (p.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY)
                pass("Device ready")
        }
    }

    private fun testPinpointPosition() = watchForChanges("pinpoint pos") {
        val p = hardware.pinpoint
        p.update()
        p.position
    }

    private fun testPinpointHeading() = watchForChanges("pinpoint pos") {
        val p = hardware.pinpoint
        p.update()
        p.getHeading(AngleUnit.RADIANS)
    }

    private fun testPinpointVel() = watchForChanges("pinpoint pos") {
        val p = hardware.pinpoint
        p.update()
        Pair(p.getVelX(DistanceUnit.INCH), p.getVelY(DistanceUnit.INCH))
    }

    override fun runOpMode() {
        hardware = CompBotHardware(hardwareMap)
        TaskSharkAndroid.setup()
        val scheduler = FastScheduler()

        val pinpointSetup = scheduler.add(put(testPinpointSetup()))
        pinpointSetup.then(put(testPinpointHeading()))
        pinpointSetup.then(put(testPinpointPosition()))
        pinpointSetup.then(put(testPinpointVel()))

        while (opModeInInit()) buildTelemetry()

        while (opModeIsActive()) {
            scheduler.tick()
            buildTelemetry()
        }
    }
}