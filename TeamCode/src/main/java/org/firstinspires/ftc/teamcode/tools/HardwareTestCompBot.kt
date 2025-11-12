package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.TaskStopException
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver

@TeleOp
class HardwareTestCompBot : HardwareTestBase() {
    data class Quadruple<P, Q, R, S>(val p: P, val q: Q, val r: R, val s: S)

    var headingMessage: String? = null
    lateinit var hardware: CompBotHardware

    fun buildTelemetry() {
        telemetry.addLine(headingMessage)
        telemetry.addLine(buildReport())

        telemetry.update()
    }

    /* -- BACKGROUND TESTS -- */

    private fun testPinpointSetup() = object : TestableAction("pinpoint status") {
        override fun onStart() {
            super.onStart()
            val p = hardware.pinpoint
            p.resetPosAndIMU()
        }

        override fun testIt() {
            val p = hardware.pinpoint
            p.update()
            reason = "${p.deviceStatus}"
            if (p.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY) try {
                pass("Device ready")
            } catch (e: TaskStopException) {
                throw e
            }
        }
    }

    private fun testPinpointPosition() = watchForChanges("pinpoint XYZ") {
        val p = hardware.pinpoint
        p.update()
        p.position
    }

    private fun testPinpointHeading() = watchForChanges("pinpoint H") {
        val p = hardware.pinpoint
        p.update()
        p.getHeading(AngleUnit.RADIANS)
    }

    private fun testPinpointVel() = watchForChanges("pinpoint vXY") {
        val p = hardware.pinpoint
        p.update()
        Pair(p.getVelX(DistanceUnit.INCH), p.getVelY(DistanceUnit.INCH))
    }

    private fun testDigitalSensor(name: String, part: DigitalChannel) = watchForChanges(name) {
        part.state
    }

    private fun testRevColorSensor(name: String, part: RevColorSensorV3) = watchForChanges(name) {
        val norm = part.normalizedColors
        Quadruple(norm.red, norm.green, norm.blue, norm.alpha)
    }

    private fun alert(message: String) = VirtualGroup {
        this.add(OneShot {
            headingMessage = message
        })
        this.add(fallingEdge { gamepad1.a })
            .then(OneShot {
                headingMessage = null
            })
    }

    private fun branch(message: String, ifTrue: VirtualGroup.Configure) =
        object : Task.Anonymous() {
            var waitFor: ITask<*>? = null
            var wasA = false
            var wasB = false

            override fun onStart() {
                headingMessage = message
            }

            override fun onTick(): Boolean {
                if (waitFor != null) {
                    if (waitFor?.getState() == ITask.State.Finished) return true
                } else {
                    val isA = gamepad1.a
                    val isB = gamepad1.b
                    if (!isA && wasA) {
                        headingMessage = null
                        // soft depend on this by polling the OneShot
                        // not amazing but it works i guess
                        waitFor = scheduler!!.add(VirtualGroup(ifTrue))
                            .then(OneShot {})
                    }
                    if (!isB && wasB) {
                        headingMessage = null
                        return true
                    }
                    wasA = isA
                    wasB = isB
                }
                return false
            }
        }

    fun testYesNo(name: String, instructions: String): TestableAction {
        return object : TestableAction(name) {
            override fun onStart() {
                super.onStart()
                headingMessage = "$instructions\nPress [A] (yes!) or [B] (no.)"
            }

            var wasA = false
            var wasB = false

            override fun testIt() {
                val isA = gamepad1.a
                val isB = gamepad1.b
                if (!isA && wasA) {
                    pass("manual")
                }
                if (!isB && wasB) {
                    fail("manual")
                }
                wasA = isA
                wasB = isB
            }
        }
    }

    override fun runOpMode() {
        hardware = CompBotHardware(hardwareMap)
        TaskSharkAndroid.setup()
        val scheduler = FastScheduler()

        // BACKGROUND ITEMS
        val pinpointSetup = scheduler.add(put(testPinpointSetup()))
        pinpointSetup.then(put(testPinpointHeading()))
        pinpointSetup.then(put(testPinpointPosition()))
        pinpointSetup.then(put(testPinpointVel()))
        scheduler.add(put(testDigitalSensor("idxMag1", hardware.idxMag1)))
        scheduler.add(put(testDigitalSensor("idxMag2", hardware.idxMag2)))
        scheduler.add(put(testDigitalSensor("idxMag3", hardware.idxMag3)))
        scheduler.add(put(testDigitalSensor("idxMag4", hardware.idxMag4)))
        scheduler.add(put(testRevColorSensor("frontColor1", hardware.frontColor1)))
        scheduler.add(put(testRevColorSensor("frontColor2", hardware.frontColor2)))
        scheduler.add(put(testRevColorSensor("backColor1", hardware.backColor1)))
        scheduler.add(put(testRevColorSensor("backColor2", hardware.backColor2)))

        // LIVE TEST SEQUENCE
        scheduler.add(
            branch("Test lights?") {
                add(OneShot { hardware.limelightLight1.position = 1.0 })
                    .then(put(testYesNo("limelightLight1", "Is the light on?")))
                    .then(OneShot {
                        hardware.limelightLight1.position = 0.0
                        hardware.limelightLight2.position = 1.0
                    })
                    .then(put(testYesNo("limelightLight2", "Is the other light on?")))
                    .then(OneShot {
                        hardware.limelightLight2.position = 0.0
                        hardware.indicator1.position = 0.5
                    })
                    .then(put(testYesNo("indicator1", "Is the light colored?")))
                    .then(OneShot {
                        hardware.indicator1.position = 0.0
                        hardware.indicator2.position = 0.5
                    })
                    .then(put(testYesNo("indicator2", "Is the other light colored?")))
                    .then(OneShot {
                        hardware.indicator2.position = 0.0
                    })
            }
        ).then(
            branch("Test indexer?") {
                add(OneShot {
                    hardware.indexer.power = 0.75
                    hardware.indexer.velocity = 250.0
                })
                    .then(put(testYesNo("indexer", "Is the indexer moving?")))
                    .then(OneShot { hardware.indexer.velocity = 0.0 })
            }
        ).then(OneShot { headingMessage = "all done" })

        while (opModeInInit()) buildTelemetry()

        while (opModeIsActive()) {
            scheduler.tick()
            buildTelemetry()
        }
    }
}