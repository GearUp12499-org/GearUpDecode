package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.tasks.DAEMON_TAGS
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

class Indexer(
    private val indexerMotor: DcMotorEx,
    private val sensor1: DigitalChannel,
    private val sensor2: DigitalChannel,
    private val sensor3: DigitalChannel,
    private val sensor4: DigitalChannel,
    private val colorFront1: RevColorSensorV3,
    private val colorFront2: RevColorSensorV3,
    private val colorBack1: RevColorSensorV3,
    private val colorBack2: RevColorSensorV3,
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
        val intakeSlot: Int = -1
    ) {
        In1(0, true, intakeSlot = 0),
        In2(2, true, intakeSlot = 1),
        In3(4, true, intakeSlot = 2),
        Out1(3, false),
        Out2(5, false),
        Out3(1, false),
        None(-1, false),
        Invalid(-2, false)
    }

    val slots = Array(3) { Slot.EMPTY }

    var lastPosition: Position = Position.None
    var resetPosition: Position = Position.None

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
        return false
    }

    fun updateState() {
//        if (!lastPosition.isIntakeStep) return
//        val d1 = colorFront1.getDistance(DistanceUnit.MM)
//        val d2 = colorFront2.getDistance(DistanceUnit.MM)
//        val minDist = min(d1, d2)
//        if (minDist >= 40) {
//            slots[lastPosition.intakeSlot] = Slot.EMPTY
//        } else {
//            val color = when {
//                d1 < 40 -> colorFront1.normalizedColors
//                d2 < 40 -> colorFront2.normalizedColors
//                else -> return
//            }
//            slots[lastPosition.intakeSlot] = if (color.blue >= 0.1) Slot.PURPLE else Slot.GREEN
//        }
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

    fun goToPosition(target: Position) = object : Anonymous() {
        private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)

        private var isInRunPos = true
        private var overshootFlip = 1
        private var targetingDirection = 1
        private var matching = false

        init {
            require(lock)
        }

        var targetTicks = 0

        override fun onStart() {
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
            indexerMotor.mode = RunMode.RUN_USING_ENCODER
//            indexerMotor.power = 0.0
            indexerMotor.power = OPERATING_POWER
            indexerMotor.velocity = indexerMotor.velocity
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
            indexerMotor.velocity = when {
                matching -> 0.0
                else -> SCAN_VELOCITY * overshootFlip * targetingDirection
            }
        }

        override fun onFinish(completedNormally: Boolean) {
            if (completedNormally) {
                indexerMotor.mode = RunMode.STOP_AND_RESET_ENCODER
                indexerMotor.power = 0.0
                resetPosition = target
                updateState()
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
            updateState()
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
}