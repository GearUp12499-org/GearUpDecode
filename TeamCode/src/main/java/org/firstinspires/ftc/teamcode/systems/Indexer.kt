package org.firstinspires.ftc.teamcode.systems

import android.graphics.Color
import android.util.Log
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.Locks
import org.firstinspires.ftc.teamcode.tasks.DAEMON_TAGS
import kotlin.math.abs
import kotlin.math.sign
import kotlin.time.Duration.Companion.seconds

class Indexer(
    private val indexerMotor: DcMotorEx,
    private val flipper: Servo,
    private val intakeMotor: DcMotor,
    private val sensor1: DigitalChannel,
    private val sensor2: DigitalChannel,
    private val sensor3: DigitalChannel,
    private val sensor4: DigitalChannel,
    private val colorFront1: RevColorSensorV3,
    private val colorFront2: RevColorSensorV3,
    private val colorBack1: RevColorSensorV3,
    private val colorBack2: RevColorSensorV3,
    private val indicator1: Servo,
    private val indicator2: Servo,
) : Task<Indexer>() {
    enum class Slot {
        EMPTY,
        GREEN,
        PURPLE
    }

    // true means activated; activation actually pulls the signal low.
    enum class Position(
        val rotationSteps: Int,
        val isIntakeStep: Boolean,
        val slot: Int = -1
    ) {
        In1(0, true, slot = 0),
        In2(2, true, slot = 1),
        In3(4, true, slot = 2),
        Out1(3, false, slot = 0),
        Out2(5, false, slot = 1),
        Out3(1, false, slot = 2),
        None(-1, false),
        Invalid(-2, false)
    }

    val slots = Array(3) { Slot.EMPTY }

    var lastPosition: Position = Position.None
    var resetPosition: Position = Position.None

    private val positionHeldFor = ElapsedTime(ElapsedTime.Resolution.SECONDS)

    companion object {
        private val LOCK_ROOT = Lock.StrLock("indexer_impl")

        /*
        0 161 323 479 642 804 963
        mean delta = 160.5
         */
        const val TICKS_PER_POSITION = 426
        const val REVOLUTION = TICKS_PER_POSITION * 6

        const val SCAN_VELOCITY = TICKS_PER_POSITION.toDouble()
        const val OPERATING_POWER = 0.7
        const val SCAN_POWER = 0.3

        const val NEARBY = TICKS_PER_POSITION / 4
        const val NOT_NEARBY = TICKS_PER_POSITION / 2

        val slotIdxToPosition = mapOf(
            0 to Position.In1,
            1 to Position.In2,
            2 to Position.In3
        )


        fun matchPosition(s1: Boolean, s2: Boolean, s3: Boolean, s4: Boolean) =
            when (s1) {
                true -> when {
                    s2 || s3 -> Position.Invalid
                    s4 -> Position.Out3
                    else -> Position.Out2
                }

                false -> when (s2) {
                    true -> when {
                        s4 -> Position.Invalid
                        s3 -> Position.In3
                        else -> Position.In1
                    }

                    false -> when {
                        s3 && s4 -> Position.Invalid
                        s3 -> Position.In2
                        s4 -> Position.Out1
                        else -> Position.None
                    }
                }
            }

        init {
            systemPackages.add(Indexer::class.qualifiedName!!)
        }
    }

    val lock = LOCK_ROOT.derive()

    init {
        require(CompBotHardware.Locks.INDEXER)
    }

    override fun getTags(): Set<String> = DAEMON_TAGS

    override fun onTick(): Boolean {
        updateState()
        return false
    }

    private fun resetStateCounter() {
        positionHeldFor.reset()
    }

    fun updateState() {
        val currentPosition =
            matchPosition(!sensor1.state, !sensor2.state, !sensor3.state, !sensor4.state)
        when (currentPosition) {
            Position.None -> {
                resetStateCounter(); return
            }

            Position.Invalid -> {
                resetStateCounter(); return
            }

            else -> if (!currentPosition.isIntakeStep) {
                resetStateCounter(); return
            }
        }
        if (positionHeldFor.time() < 0.25) return
        val f1 = colorFront1.normalizedColors
        val f2 = colorFront2.normalizedColors
        val f1Hsv = FloatArray(3)
        val f2Hsv = FloatArray(3)
        Color.RGBToHSV(
            (f1.red * 255).toInt(),
            (f1.green * 255).toInt(),
            (f1.blue * 255).toInt(),
            f1Hsv
        )
        Color.RGBToHSV(
            (f2.red * 255).toInt(),
            (f2.green * 255).toInt(),
            (f2.blue * 255).toInt(),
            f2Hsv
        )
        val d1 = colorFront1.getDistance(DistanceUnit.MM)
        val d2 = colorFront2.getDistance(DistanceUnit.MM)

        if (d1 < 15 || (d1 < 40 && d2 >= 40)) {
            // use d1
            slots[currentPosition.slot] = Slot.PURPLE
        } else if (d2 < 40) {
            // use d2
            slots[currentPosition.slot] = Slot.PURPLE
        } else {
            // nothing
            slots[currentPosition.slot] = Slot.EMPTY
        }
    }

    fun deleteCurrent() {
        slots[lastPosition.slot] = Slot.EMPTY
    }

    fun shoot() = object : Wait(0.25.seconds) {
        override fun onStart() {
            super.onStart()
            flipper.position = CompBotHardware.FLIPPER_UP
        }

        override fun onFinish(completedNormally: Boolean) {
            super.onFinish(completedNormally)
            flipper.position = CompBotHardware.FLIPPER_DOWN
            deleteCurrent()
        }
    }

    override fun onFinish(completedNormally: Boolean) {
        indexerMotor.power = 0.0
    }

    fun Position.getTicks(): Int {
        if (resetPosition.rotationSteps < 0) // invalid
            throw IllegalStateException("Indexer has never been synchronized!")
        val offsetSteps = (rotationSteps - resetPosition.rotationSteps).mod(6)
        Log.i(
            "indexer",
            "Offset steps $this to $resetPosition: $offsetSteps ($rotationSteps - ${resetPosition.rotationSteps} mod 6)"
        )
        return offsetSteps * TICKS_PER_POSITION * -1
    }

    fun goToPosition(target: Position) = goToPosition { target }

    fun goToPosition(targetProvider: () -> Position) = object : Anonymous() {
        private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)

        private var isInRunPos = true
        private var overshootFlip = 1
        private var targetingDirection = 1
        private var matching = false
        private lateinit var target: Position

        init {
            require(lock)
        }

        var targetTicks = 0

        override fun onStart() {
            this.target = targetProvider()
            lastPosition = target
            val targetRelativeTicks = target.getTicks()
            // there's probably a better way of doing this but i'm too tired atm
            var middle = indexerMotor.currentPosition.floorDiv(REVOLUTION) * REVOLUTION
            var upper = middle + REVOLUTION
            var lower = middle - REVOLUTION

            lower += targetRelativeTicks
            middle += targetRelativeTicks
            upper += targetRelativeTicks

            val rightNow = indexerMotor.currentPosition
            val lowerDelta = abs(lower - rightNow)
            val middleDelta = abs(middle - rightNow)
            val upperDelta = abs(upper - rightNow)
            targetTicks = when {
                lowerDelta < middleDelta && lowerDelta < upperDelta -> lower
                upperDelta < middleDelta && upperDelta < lowerDelta -> upper
                else -> middle
            }
            Log.i(
                "indexer",
                "low %d med %d high %d target %d ticks %d | chose ".format(
                    lower,
                    middle,
                    upper,
                    rightNow,
                    targetRelativeTicks
                )
            )
            isInRunPos = true
            beforeRunToPos()
        }

        override fun onTick(): Boolean {
            // if we _see_ the target position and we're reasonably close, try to be more precise with it
            val instant =
                matchPosition(!sensor1.state, !sensor2.state, !sensor3.state, !sensor4.state)
            val error = abs(indexerMotor.currentPosition - targetTicks)

            if (isInRunPos) tickRunToPos(instant, error)
            else tickRunSensors(instant, error)

            Log.d(
                "indexer",
                "%s overshoot $overshootFlip ==? $matching result $instant deltaTicks $error MOTOR STATUS ${indexerMotor.mode} p${indexerMotor.power} v${indexerMotor.velocity} t${indexerMotor.targetPosition}".format(
                    if (isInRunPos) "runPos" else "runSensor"
                )
            )

            return matching && timer.time() > 0.5
        }

        fun beforeRunToPos() {
            val now = indexerMotor.currentPosition
            indexerMotor.targetPosition = targetTicks
            indexerMotor.mode = RunMode.RUN_TO_POSITION
            indexerMotor.power = OPERATING_POWER
            targetingDirection = (targetTicks - now).sign
            overshootFlip = 1
        }

        fun tickRunToPos(instant: Position, error: Int) {
            if (error < NEARBY) {
                isInRunPos = false
                beforeRunSensors()
                return tickRunSensors(instant, error)
            }
        }

        fun beforeRunSensors() {
            indexerMotor.mode = RunMode.RUN_WITHOUT_ENCODER
//            indexerMotor.power = 0.0
            indexerMotor.power = SCAN_POWER * targetingDirection
        }

        fun tickRunSensors(instant: Position, error: Int) {
            // we've messed it up.
            if (error > NOT_NEARBY) {
                isInRunPos = true
                beforeRunToPos()
                return tickRunToPos(instant, error)
            }

            val isNow = instant == target
            // ooh!
            if (isNow && !matching) timer.reset()
            // we went past it.
            if (!isNow && matching) overshootFlip = -overshootFlip

            matching = isNow
            indexerMotor.power = when {
                matching -> 0.0
                else -> SCAN_POWER * overshootFlip * targetingDirection
            }
        }

        override fun onFinish(completedNormally: Boolean) {
            if (completedNormally) {
                indexerMotor.mode = RunMode.STOP_AND_RESET_ENCODER
                indexerMotor.power = 0.0
                resetPosition = target
            }
        }
    }

    fun syncPosition() = object : Anonymous() {
        private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)
        private var lastPos = Position.None

        override fun onStart() {
            indexerMotor.mode = RunMode.RUN_USING_ENCODER
            indexerMotor.power = 1.0
            indexerMotor.velocity = SCAN_VELOCITY
            timer.reset()
        }

        override fun onFinish(completedNormally: Boolean) {
            indexerMotor.power = 0.0
            indexerMotor.mode = RunMode.STOP_AND_RESET_ENCODER
            indexerMotor.mode = RunMode.RUN_USING_ENCODER
        }

        override fun onTick(): Boolean {
            val position =
                matchPosition(!sensor1.state, !sensor2.state, !sensor3.state, !sensor4.state)
            when (position) {
                Position.Invalid, Position.None -> {
                    indexerMotor.velocity = SCAN_VELOCITY
                    return false
                }

                else -> {
                    indexerMotor.velocity = 0.0
                    if (lastPos != position) timer.reset()
                }
            }
            lastPos = position
            if (timer.time() > 0.5) {
                lastPosition = position
                resetPosition = position
                return true
            }
            return false
        }

        init {
            require(lock)
        }
    }

    fun intake() = object : Anonymous() {
        init {
            require(Locks.INTAKE)
        }

        private var subTask: ITask<*>? = null
        private var done = false
        private var slotN = -1
        private var slotTimer = false
        private val timer = ElapsedTime()
        private val timer2 = ElapsedTime()

        override fun onStart() {
            if (slots.all { it != Slot.EMPTY }) finish()
            intakeMotor.power = CompBotHardware.INTAKE_POWER
        }

        override fun onTick(): Boolean {
            if (slots.all { it != Slot.EMPTY }) {
                if (!done) {
                    timer.reset()
                    done = true
                }
                if (timer.time() > 0.25) return true
                return false
            } else {
                done = false
            }
            val slot = slots.indexOfFirst { it == Slot.EMPTY }
            val slotPos = slotIdxToPosition[slot]!!
            val isTaskFree = (subTask == null || subTask!!.getState() == ITask.State.Finished || subTask!!.getState() == ITask.State.Cancelled)
            indicator1.position = if (isTaskFree) 0.5 else 0.8
            indicator2.position = if (isTaskFree) 0.5 else 0.8
            if (slot != slotN) {
                if (!slotTimer) {
                    timer2.reset()
                    slotTimer = true
                }
                if (timer2.time() > 0.2) {
                    if (lastPosition != slotPos && isTaskFree) {
                        slotTimer = false
                        subTask = scheduler!!.add(
                            goToPosition(slotPos)
                        )
                    }
                }
            }

            return false
        }

        override fun onFinish(completedNormally: Boolean) {
            subTask?.let {
                if (it.getState() == ITask.State.Ticking) it.stop()
            }
            intakeMotor.power = 0.0
            indicator1.position = 0.0
            indicator2.position = 0.0
        }
    }
}