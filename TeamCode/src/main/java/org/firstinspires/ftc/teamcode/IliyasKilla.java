package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.swervely.PinpointSetup;

/**
 * the main driving program for the robot. it shows up on the driver hub as "IliyasKilla".
 *
 * how to drive:
 *   left stick  - move the robot in any direction (forward, back, sideways, diagonal)
 *   right stick - turn the robot left or right
 *   arrow pad   - move slowly and perfectly straight in one direction (good for lining up)
 *   right bumper - hold for slow mode: everything moves at SlowScale (careful lining up)
 *   options     - tell the robot "the way you are facing right now is forward"
 *
 * every number marked "public static" below can be changed live from the dashboard website
 * while the robot is running, without re-uploading the code.
 */
@Config
@TeleOp(name = "IliyasKilla", group = "TeleOp")
public class IliyasKilla extends LinearOpMode {
    /*
     * HEADING HOLD
     *
     * "heading" means the direction the robot is facing, like a compass.
     *
     * a robot never drives perfectly straight on its own. one side always pushes a little harder
     * than the other (a stronger motor, a stiffer gearbox, more weight on one corner), so when you
     * push the stick straight forward the robot slowly twists to one side.
     *
     * heading hold fixes that. it remembers which way the robot was facing, and while you drive it
     * keeps checking. if the robot has twisted away, it adds a small turn the other way to bring it
     * back. you dont see it working, the robot just drives straight.
     *
     * the next three numbers decide how that correction feels. they are called p, i and d.
     */

    /*
     * how hard the robot turns back when it is facing the wrong way.
     * the further off it is, the harder it turns back, and this number scales that.
     *   too low:  the robot still drifts, because the correction is too weak to beat the twist
     *   too high: the robot overshoots and wiggles left-right-left while driving
     * if it still twists while driving, raise it (try 1.0, then 1.5). if it wiggles, lower it.
     */
    public static double HeadingKp = 2.0;

    /*
     * fixes a small error that wont go away by itself.
     * if the robot sits slightly off for a while, this slowly adds more and more correction until
     * it lines up. keep it small, a big value makes the robot swing back and forth slowly.
     * usually you dont need to touch this.
     */
    public static double HeadingKi = 0.03;

    /*
     * the brakes on the correction.
     * when the robot is already turning back toward the right direction, this eases off the
     * correction so it arrives smoothly and doesnt swing past.
     *   too low:  the robot swings past the right direction and comes back (wobble)
     *   too high: the correction feels jittery or twitchy
     */
    public static double HeadingKd = 0.4;

    /*
     * which way the correction turns. this is either 1 or -1, nothing else.
     *
     * the robot's compass might count "turning left" as going up or going down, depending on how
     * the sensor is mounted. if this number is wrong, every correction turns the robot further the
     * wrong way: it twists a bit, the correction makes it twist more, and it ends up spinning on
     * its own. if you ever see the robot slowly spin by itself when you let go, flip this
     * (1 becomes -1, -1 becomes 1). we tested it and 1 is correct for this robot right now.
     */
    public static double HeadingSign = 1.0;

    /*
     * the most turning power heading hold is allowed to use, from 0 to 1.
     * 0 turns heading hold off completely. 1 would let it use full power, which is too much.
     * this cap means even if something goes wrong, heading hold can only turn the robot gently.
     */
    public static double HeadingMax = 0.5;

    /*
     * how many degrees off the robot can be before heading hold bothers correcting.
     * without this, the robot would make tiny constant corrections and twitch while standing still.
     */
    public static double HeadingTolerance = 2.0;

    /*
     * how many seconds to wait after you let go of the turn stick before heading hold takes over.
     * the robot keeps rolling a little after you stop turning. if heading hold locked in the
     * direction instantly, it would grab that half-turned direction and then snap the robot back.
     * waiting lets the robot finish its turn first, then it holds the new direction.
     */
    public static double TurnSettle = 0.3;

    /*
     * lets heading hold take over sooner when the robot has actually stopped turning, instead of
     * always waiting the full TurnSettle. once the robot turns slower than this many degrees a
     * second (the pinpoint measures it), the turn counts as finished. TurnSettle stays the
     * longest it ever waits. 0 turns this off (always wait TurnSettle). try 20.
     */
    public static double SettledRate = 0;

    /*
     * how many degrees the "Heading step" button on the laptop turns the direction heading hold
     * aims for. pressing it is a step test: watch the Heading and Target Heading graphs to see if
     * heading hold gets there quickly, overshoots or wiggles, then tune HeadingKp, HeadingKi and
     * HeadingKd.
     */
    public static double StepDegrees = 90;

    /*
     * POSITION HOLD (currently turned off)
     *
     * when you let go of the sticks, position hold remembers the spot where the robot stopped. if
     * something bumps the robot off that spot, it drives back to it, like it is anchored there.
     *
     * it is turned off right now (PositionMax is 0) because it was making the robot creep away on
     * its own. the sensor that tracks the robot's position wasnt agreeing with the wheels about
     * which way is which, so instead of pushing back it pushed further away.
     *
     * the X numbers are for forward/backward, the Y numbers are for left/right.
     * Kp, Ki and Kd work the same way as in heading hold above: strength, slow fix, and brakes.
     */
    public static double XKp = 0.025;
    public static double XKi = 0.0;
    public static double XKd = 0.005;

    public static double YKp = 0.025;
    public static double YKi = 0.0;
    public static double YKd = 0.005;

    /*
     * the most driving power position hold is allowed to use, from 0 to 1.
     * 0 means position hold is off. leave it at 0 unless position hold gets fixed.
     */
    public static double PositionMax = 0.0;

    /*
     * DRIVER SETTINGS
     */

    /*
     * which way sliding sideways goes. this is either 1 or -1.
     * mecanum wheels can be put on in two patterns, and the code has to know which one this robot
     * has. if you push the left stick right and the robot slides left, flip this.
     */
    public static double StrafeSign = -1.0;

    /*
     * how far the stick has to be pushed (from 0 to 1) before the robot reacts.
     * controller sticks rarely sit perfectly in the middle when you let go. they rest a tiny bit
     * off, and the robot would treat that as you pushing it and slowly creep. anything smaller
     * than this number is treated as "not touching the stick". raise it if the robot still creeps
     * with nobody touching the controller.
     */
    public static double DriveDeadzone = 0.1;

    /*
     * how fast the arrow pad drives the robot, from 0 to 1 (1 = full speed).
     * the arrow pad is for slow, careful moves like lining up, so this is kept low.
     */
    public static double DpadSpeed = 0.4;

    /*
     * how many seconds after you let go of everything before the robot counts as "stopped" and
     * remembers its spot. a short wait so a quick let-go between moves doesnt count.
     */
    public static double HoldDelay = 0.15;

    /*
     * how fast everything goes while the right bumper is held (slow mode), from 0 to 1.
     * for careful lining up without letting go of the sticks.
     */
    public static double SlowScale = 0.35;

    /*
     * how strong the turn stick is, from 0 to 1. lower it if full turn spins the robot too fast
     * to aim.
     */
    public static double TurnScale = 1.0;

    /*
     * how the turn stick builds up. 1 is straight (half the stick = half the turn). 2 makes small
     * pushes much gentler for aiming while full push stays full speed. try 1.5 to 2.
     */
    public static double TurnCurve = 1.0;

    /*
     * smoother sticks. the deadzone above is round instead of square (small diagonal pushes no
     * longer snap to straight forward or sideways), and power grows from zero at the edge of the
     * deadzone instead of jumping straight to 10%. false keeps the old feel.
     */
    public static boolean SmoothSticks = false;

    /*
     * true: up on the stick always means away from the driver (field centric). false: up means
     * the way the robot faces (robot centric), like most simple robots.
     */
    public static boolean FieldCentric = true;

    /*
     * keeps the robot the same speed as the battery drains: wheel powers are scaled up a little
     * when the battery is below NominalVoltage. off by default.
     */
    public static boolean VoltageComp = false;
    public static double NominalVoltage = 12.5;

    // the four wheel motors and the pinpoint (the little box that tracks where the robot is and which way it faces)
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor RL;
    private DcMotor RR;

    private GoBildaPinpointDriver Pinpoint;

    // one calculator each for holding forward/back, left/right and the facing direction
    private final Pid XPid = new Pid();
    private final Pid YPid = new Pid();
    private final Pid HeadingPid = new Pid();

    // where the robot is trying to stay and which way it is trying to face
    private double TargetX;
    private double TargetY;
    private double TargetHeading;

    private final ElapsedTime IdleTimer = new ElapsedTime();
    private ElapsedTime TurnTimer = new ElapsedTime();
    private boolean HoldActive;

    private long LastTime;

    private VoltageSensor Battery;

    /**
     * runs when you press init and start on the driver hub. sets everything up, then keeps
     * reading the controller and powering the wheels over and over until you press stop.
     */
    @Override
    public void runOpMode() {
        // find the motors and the pinpoint using the names saved in the robot configuration
        FL = hardwareMap.get(DcMotor.class, "fl");
        FR = hardwareMap.get(DcMotor.class, "fr");
        RL = hardwareMap.get(DcMotor.class, "rl");
        RR = hardwareMap.get(DcMotor.class, "rr");

        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // the control hub's own battery reading, for VoltageComp
        Battery = hardwareMap.voltageSensor.iterator().next();

        // the motors on each side are mounted facing opposite ways, so one side has to be reversed
        // for "positive power" to mean "wheel rolls forward" on all four
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        // brake = the wheels lock up when they get no power, instead of rolling freely
        for (DcMotor Motor : new DcMotor[]{FL, FR, RL, RR}) {
            Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // tell the pinpoint where its tracking wheels sit on the robot, what kind they are and which
        // way each one counts, then zero it so the spot where the robot starts counts as the
        // starting point. the numbers live in PinpointSetup so every program uses the same ones.
        // on this robot both wheels are mounted so they count backwards (we tested it: driving
        // forward read as going backward, sliding right read as going left), which is why
        // PinpointSetup says REVERSED for both. if they were wrong, position hold would push the
        // robot the wrong way
        PinpointSetup.apply(Pinpoint);

        Pinpoint.resetPosAndIMU();

        // the dashboard normally sends the robot's numbers to the laptop in bundles every 100 ms,
        // which makes the field drawing and graphs look choppy (only 10 updates a second).
        // sending every 25 ms gives 40 smooth updates a second
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        // buttons the laptop shows while this program runs. a press runs when the driving loop
        // below sends its numbers to the laptop, so it never lands halfway through working out the
        // wheel powers. presses made before start are thrown away, so nothing moves by surprise
        FtcDashboard.getInstance().addAction("Reset heading", "Takes the way the robot faces now as forward for driving. Does not move it.", this::ResetHeading);
        FtcDashboard.getInstance().addAction("Hold position here", "Holds the robot on the spot it stands on now. Only pushes back while PositionMax is above 0.", this::StartHold);
        FtcDashboard.getInstance().addAction("Heading step", "Turns the robot by StepDegrees and holds it there, to see how well turning settles. Moves the robot.", this::StepHeading);

        telemetry.addLine("Ready");
        telemetry.update();

        // wait here until the driver presses start
        waitForStart();

        if (isStopRequested()) return;

        // the direction the robot faces at the start is the direction heading hold keeps it at,
        // until the driver turns it somewhere else
        Pinpoint.update();
        TargetHeading = Pinpoint.getHeading(AngleUnit.RADIANS);
        StartHold();

        // everything below repeats many times per second until stop is pressed
        while (opModeIsActive()) {
            // ask the pinpoint where the robot is (in inches) and which way it faces
            Pinpoint.update();

            double X = Pinpoint.getPosX(DistanceUnit.INCH);
            double Y = Pinpoint.getPosY(DistanceUnit.INCH);
            double Heading = Pinpoint.getHeading(AngleUnit.RADIANS);

            // options button: whatever way the robot faces right now becomes "forward" for the sticks
            if (gamepad1.options) {
                ResetHeading();
                Heading = 0;
            }

            // measure how much time passed since the last repeat. the calculators need it,
            // and if the number looks broken we use a normal value instead
            long Now = System.nanoTime();
            double Dt = (Now - LastTime) / 1e9;
            double LoopMs = Dt * 1000;

            LastTime = Now;

            if (Dt < 0.005 || Dt > 0.05) Dt = 0.02;

            // read the controller. the stick reports "up" as a negative number, so we flip it
            double DriveX;
            double DriveY;

            if (SmoothSticks) {
                // a round deadzone, with power growing from zero at its edge
                double StickX = gamepad1.left_stick_x;
                double StickY = -gamepad1.left_stick_y;
                double Push = Math.min(Math.hypot(StickX, StickY), 1);
                double Scale = Push < DriveDeadzone ? 0 : (Push - DriveDeadzone) / (1 - DriveDeadzone) / Push;

                DriveX = StickX * Scale;
                DriveY = StickY * Scale;
            } else {
                DriveX = Deadzone(gamepad1.left_stick_x);
                DriveY = Deadzone(-gamepad1.left_stick_y);
            }

            double Turn = Deadzone(gamepad1.right_stick_x);

            if (SmoothSticks && Turn != 0) Turn = Math.signum(Turn) * (Math.abs(Turn) - DriveDeadzone) / (1 - DriveDeadzone);

            // the turn stick's curve and strength: gentle small pushes for aiming
            Turn = Math.signum(Turn) * Math.pow(Math.abs(Turn), TurnCurve) * TurnScale;

            // slow mode: hold the right bumper to move and turn at SlowScale
            boolean Slow = gamepad1.right_bumper;

            if (Slow) {
                DriveX *= SlowScale;
                DriveY *= SlowScale;
                Turn *= SlowScale;
            }

            // arrow pad: right/left and up/down each become a fixed slow speed
            double DpadX = ((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0)) * DpadSpeed;
            double DpadY = ((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0)) * DpadSpeed;

            // if the driver is touching anything, the driver is in control. once they let go of
            // everything for a moment, remember the spot the robot stopped at
            if (DriveX != 0 || DriveY != 0 || Turn != 0 || DpadX != 0 || DpadY != 0) {
                IdleTimer.reset();
                HoldActive = false;

                XPid.Integral = 0;
                YPid.Integral = 0;
                HeadingPid.Integral = 0;
            } else if (!HoldActive && IdleTimer.seconds() >= HoldDelay) {
                StartHold();
            }

            // while the driver is turning (and shortly after), keep updating which way the robot
            // should face, so heading hold doesnt fight the driver
            if (Turn != 0) TurnTimer.reset();

            // the turn has finished once TurnSettle has passed, or sooner (with SettledRate on)
            // once the robot has really stopped turning. a tenth of a second always passes first,
            // so letting go mid-spin isnt mistaken for stopped
            double TurnRate = Math.abs(Pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            boolean Settled = TurnTimer.seconds() >= TurnSettle || (SettledRate > 0 && Turn == 0 && TurnTimer.seconds() >= 0.1 && TurnRate < SettledRate);

            if (Settled && TurnTimer.seconds() < TurnSettle) {
                // count the turn as over, so the hold below starts now
                TurnTimer = new ElapsedTime(System.nanoTime() - (long) (TurnSettle * 1e9));
            }

            if (TurnTimer.seconds() < TurnSettle) {
                TargetHeading = Heading;
                HeadingPid.Reset();
            }

            // position hold: how hard to push the robot back to its saved spot (zero while it is turned off)
            double CorrectionForward = 0;
            double CorrectionLeft = 0;

            if (HoldActive) {
                CorrectionForward = Range.clip(XPid.Update(TargetX - X, Dt, XKp, XKi, XKd, 1.0), -PositionMax, PositionMax);
                CorrectionLeft = Range.clip(YPid.Update(TargetY - Y, Dt, YKp, YKi, YKd, 1.0), -PositionMax, PositionMax);
            }

            // turning: use the driver's turn stick. if the driver isnt turning, heading hold
            // gently turns the robot back to the direction it is supposed to face
            double Rotation = Turn;

            if (TurnTimer.seconds() >= TurnSettle) {
                double HeadingError = AngleUnit.normalizeRadians(TargetHeading - Heading);
                double Output = HeadingPid.Update(HeadingError, Dt, HeadingKp, HeadingKi, HeadingKd, 0.25);

                if (Math.abs(Math.toDegrees(HeadingError)) < HeadingTolerance) {
                    HeadingPid.Integral = 0;
                    Output = 0;
                }

                Rotation = Range.clip(-HeadingSign * Output, -HeadingMax, HeadingMax);
            }

            // field centric: "up" on the stick always means away from the driver, no matter
            // which way the robot is turned. we rotate the stick direction by the robot's
            // facing to work out which way that is for the robot itself
            double FieldX = DriveX - CorrectionLeft;
            double FieldY = DriveY + CorrectionForward;

            double Magnitude = Math.hypot(FieldX, FieldY);

            if (Magnitude > 1) {
                FieldX /= Magnitude;
                FieldY /= Magnitude;
            }

            // robot centric drives as if the robot always faced forward
            double Facing = FieldCentric ? Heading : 0;
            double Cos = Math.cos(-Facing);
            double Sin = Math.sin(-Facing);

            // the arrow pad is added after the rotation, so it always moves relative to the robot itself
            double RobotX = (FieldX * Cos - FieldY * Sin + DpadX) * StrafeSign;
            double RobotY = FieldX * Sin + FieldY * Cos + DpadY;

            // mecanum wheels: mixing forward, sideways and turning into a power for each wheel.
            // if any wheel would need more than full power, all four are scaled down together
            // so the robot still moves in the right direction
            double Denominator = Math.max(Math.abs(RobotX) + Math.abs(RobotY) + Math.abs(Rotation), 1);

            double PowerFL = (RobotY + RobotX + Rotation) / Denominator;
            double PowerRL = (RobotY - RobotX + Rotation) / Denominator;
            double PowerFR = (RobotY - RobotX - Rotation) / Denominator;
            double PowerRR = (RobotY + RobotX - Rotation) / Denominator;

            // a weaker battery gets a little more power, so the robot feels the same all match
            // (only read when used: asking the hub costs the loop a couple of milliseconds)
            double Volts = VoltageComp ? Battery.getVoltage() : 0;

            if (VoltageComp && Volts > 6) {
                double Boost = NominalVoltage / Volts;

                PowerFL = Range.clip(PowerFL * Boost, -1, 1);
                PowerRL = Range.clip(PowerRL * Boost, -1, 1);
                PowerFR = Range.clip(PowerFR * Boost, -1, 1);
                PowerRR = Range.clip(PowerRR * Boost, -1, 1);
            }

            FL.setPower(PowerFL);
            RL.setPower(PowerRL);
            FR.setPower(PowerFR);
            RR.setPower(PowerRR);

            // text shown on the driver hub screen, useful for checking what the robot thinks is happening
            telemetry.addData("Pose", "%.2f, %.2f, %.1f", X, Y, Math.toDegrees(Heading));
            telemetry.addData("Target", "%.2f, %.2f, %.1f", TargetX, TargetY, Math.toDegrees(TargetHeading));
            telemetry.addData("Error", "%.2f, %.2f, %.1f", TargetX - X, TargetY - Y, Math.toDegrees(AngleUnit.normalizeRadians(TargetHeading - Heading)));

            telemetry.addData("Mode", (HoldActive ? "Hold" : "Driver") + (Slow ? ", slow" : "") + (FieldCentric ? "" : ", robot centric"));
            telemetry.addData("Loop", "%.1f ms", LoopMs);
            telemetry.addData("Correction", "fwd %.3f  left %.3f  rot %.3f", CorrectionForward, CorrectionLeft, Rotation);
            telemetry.addData("Pinpoint", Pinpoint.getDeviceStatus());

            telemetry.update();

            // the same numbers sent to the dashboard website, where they can be graphed,
            // plus a drawing of the robot on the field map
            TelemetryPacket Packet = new TelemetryPacket();

            Packet.put("X", X);
            Packet.put("Y", Y);
            Packet.put("Heading", Math.toDegrees(Heading));

            Packet.put("Target X", TargetX);
            Packet.put("Target Y", TargetY);
            Packet.put("Target Heading", Math.toDegrees(TargetHeading));

            Packet.put("Correction Forward", CorrectionForward);
            Packet.put("Correction Left", CorrectionLeft);
            Packet.put("Rotation", Rotation);

            Packet.put("Drive X", DriveX);
            Packet.put("Drive Y", DriveY);
            Packet.put("Turn", Turn);
            Packet.put("Turn Rate", TurnRate);
            Packet.put("Slow", Slow ? 1 : 0);
            Packet.put("Loop ms", LoopMs);
            if (VoltageComp) Packet.put("Battery", Volts);

            Packet.put("FL", PowerFL);
            Packet.put("FR", PowerFR);
            Packet.put("RL", PowerRL);
            Packet.put("RR", PowerRR);

            Packet.fieldOverlay()
                    .setStroke("#3F51B5")
                    .strokeCircle(X, Y, 9)
                    .strokeLine(X, Y, X + 9 * Math.cos(Heading), Y + 9 * Math.sin(Heading))
                    .setStroke("#4CAF50")
                    .strokeCircle(TargetX, TargetY, 2);

            FtcDashboard.getInstance().sendTelemetryPacket(Packet);
        }
    }

    /**
     * remembers the spot where the robot is right now, so position hold knows where to keep it.
     * it does not touch the facing direction: that only changes when the driver turns, so if the
     * robot twisted a bit while driving, heading hold still turns it back after you let go.
     */
    private void StartHold() {
        TargetX = Pinpoint.getPosX(DistanceUnit.INCH);
        TargetY = Pinpoint.getPosY(DistanceUnit.INCH);

        XPid.Reset();
        YPid.Reset();

        LastTime = System.nanoTime();
        HoldActive = true;
    }

    /**
     * whatever way the robot faces right now becomes "forward" for the sticks, and heading hold
     * aims for it. used by the options button and the "Reset heading" button on the laptop.
     */
    private void ResetHeading() {
        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Pinpoint.getPosX(DistanceUnit.INCH), Pinpoint.getPosY(DistanceUnit.INCH), AngleUnit.RADIANS, 0));

        TargetHeading = 0;
        HeadingPid.Reset();
    }

    /**
     * the "Heading step" button on the laptop: turns the direction heading hold aims for by
     * StepDegrees, straight away. if the driver is holding the turn stick, the driver still wins.
     */
    private void StepHeading() {
        TargetHeading = AngleUnit.normalizeRadians(TargetHeading + Math.toRadians(StepDegrees));
        HeadingPid.Reset();

        // heading hold normally waits TurnSettle seconds after the turn stick is let go, and
        // until then it keeps copying the current facing into TargetHeading, which would undo the
        // step. a timer that started TurnSettle seconds ago lets it take over on the next repeat
        TurnTimer = new ElapsedTime(System.nanoTime() - (long) (TurnSettle * 1e9));
    }

    /**
     * ignores tiny stick movements. controller sticks often dont sit perfectly in the middle,
     * and without this the robot would creep around on its own.
     *
     * @param Value how far the stick is pushed, from -1 to 1
     * @return the same value, or 0 if the stick is barely pushed
     */
    private static double Deadzone(double Value) {
        return Math.abs(Value) < DriveDeadzone ? 0 : Value;
    }

    /**
     * a pid calculator: given how far off the robot is, it works out how hard to push it back.
     * it uses three parts: how far off it is now (p), how long it has been off (i),
     * and how fast it is already moving back (d).
     */
    private static class Pid {
        double Integral;
        double LastError;

        /**
         * works out the next push from the current mistake.
         *
         * @param Error how far off the robot is (target minus where it actually is)
         * @param Dt seconds since this was last called
         * @param Kp strength of the push based on how far off it is
         * @param Ki strength of the push that builds up over time
         * @param Kd strength of the calming effect when it is already moving back
         * @param IntegralLimit cap on the built up part so it cant grow forever
         * @return how hard to push, where bigger numbers mean a stronger push
         */
        double Update(double Error, double Dt, double Kp, double Ki, double Kd, double IntegralLimit) {
            Integral = Range.clip(Integral + Error * Dt, -IntegralLimit, IntegralLimit);

            double Derivative = (Error - LastError) / Dt;
            LastError = Error;

            return Kp * Error + Ki * Integral + Kd * Derivative;
        }

        /**
         * forgets everything the calculator remembered, so it starts fresh.
         */
        void Reset() {
            Integral = 0;
            LastError = 0;
        }
    }
}
