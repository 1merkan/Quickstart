package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestSets3", group = "TeleOp")
public class TestSets3 extends LinearOpMode {

    // ============================================================
    // MOTORS
    // ============================================================

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor RL;
    private DcMotor RR;

    // ============================================================
    // PINPOINT
    // ============================================================

    private GoBildaPinpointDriver pinpoint;


    // ============================================================
    // HEADING CONTROLLER
    //
    // Мягкие значения.
    // I специально оставлен 0.
    // ============================================================

    public static double HEADING_KP = 0.75;
    public static double HEADING_KI = 0.03;
    public static double HEADING_KD = 0.4;

    /*
     * Если робот корректирует угол В НЕПРАВИЛЬНУЮ сторону:
     *
     *  1.0 -> нормальный знак
     * -1.0 -> обратный знак
     */
    public static double HEADING_SIGN = 1.0;

    /*
     * Очень маленький максимум.
     *
     * Это специально, чтобы PID
     * физически не мог резко развернуть базу.
     */
    public static double HEADING_MAX = 0.12;

    /*
     * В пределах этого угла
     * вообще не исправляем heading.
     */
    public static double HEADING_TOLERANCE = 2.0;


    // ============================================================
    // POSITION CONTROLLER
    // ============================================================

    public static double X_KP = 0.025;
    public static double X_KI = 0.0;
    public static double X_KD = 0.005;

    public static double Y_KP = 0.025;
    public static double Y_KI = 0.0;
    public static double Y_KD = 0.005;

    /*
     * Максимальная коррекция позиции.
     */
    public static double POSITION_MAX = 0.20;


    // ============================================================
    // DRIVER
    // ============================================================

    public static double DRIVE_DEADZONE = 0.06;

    /*
     * Если водитель отпустил стики,
     * робот начинает hold.
     */
    public static double HOLD_DELAY = 0.15;


    // ============================================================
    // TARGET
    // ============================================================

    private double targetX;
    private double targetY;
    private double targetHeading;


    // ============================================================
    // PID STATE
    // ============================================================

    private double headingIntegral = 0.0;
    private double headingLastError = 0.0;

    private double xIntegral = 0.0;
    private double xLastError = 0.0;

    private double yIntegral = 0.0;
    private double yLastError = 0.0;

    private long lastTime;


    // ============================================================
    // HOLD STATE
    // ============================================================

    private boolean holdActive = true;

    private long driverLastActiveTime;


    // ============================================================
    // RUN OPMODE
    // ============================================================

    @Override
    public void runOpMode() {

        // ========================================================
        // HARDWARE
        // ========================================================

        FL = hardwareMap.get(
                DcMotor.class,
                "fl"
        );

        FR = hardwareMap.get(
                DcMotor.class,
                "fr"
        );

        RL = hardwareMap.get(
                DcMotor.class,
                "rl"
        );

        RR = hardwareMap.get(
                DcMotor.class,
                "rr"
        );

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );


        // ========================================================
        // MOTOR DIRECTIONS
        // ========================================================

        FL.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        RL.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        FR.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        RR.setDirection(
                DcMotorSimple.Direction.FORWARD
        );


        // ========================================================
        // MOTOR MODES
        // ========================================================

        FL.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        FR.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        RL.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        RR.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );


        // ========================================================
        // BRAKE
        // ========================================================

        FL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        FR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RL.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        RR.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );


        // ========================================================
        // PINPOINT
        // ========================================================

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();


        // ========================================================
        // READY
        // ========================================================

        telemetry.addLine("==============================");
        telemetry.addLine("       TESTSETS3");
        telemetry.addLine("==============================");
        telemetry.addLine("Pedro-style Hold Controller");
        telemetry.addLine("Field Centric");
        telemetry.addLine("Heading Hold");
        telemetry.addLine("Position Hold");
        telemetry.addLine("");
        telemetry.addLine("No buttons required");
        telemetry.update();


        waitForStart();

        if (isStopRequested()) {
            return;
        }


        // ========================================================
        // INITIAL TARGET
        // ========================================================

        pinpoint.update();

        targetX =
                pinpoint.getPosX(
                        DistanceUnit.INCH
                );

        targetY =
                pinpoint.getPosY(
                        DistanceUnit.INCH
                );

        targetHeading =
                pinpoint.getHeading(
                        AngleUnit.RADIANS
                );


        resetControllers();

        holdActive = true;

        driverLastActiveTime =
                System.currentTimeMillis();


        // ========================================================
        // MAIN LOOP
        // ========================================================

        while (opModeIsActive()) {

            // ====================================================
            // PINPOINT
            // ====================================================

            pinpoint.update();


            // ====================================================
            // CURRENT POSE
            // ====================================================

            double currentX =
                    pinpoint.getPosX(
                            DistanceUnit.INCH
                    );

            double currentY =
                    pinpoint.getPosY(
                            DistanceUnit.INCH
                    );

            double currentHeading =
                    pinpoint.getHeading(
                            AngleUnit.RADIANS
                    );


            // ====================================================
            // TIME
            // ====================================================

            long now =
                    System.nanoTime();

            double dt =
                    (now - lastTime)
                    / 1_000_000_000.0;

            lastTime = now;


            if (dt < 0.005 || dt > 0.05) {
                dt = 0.02;
            }


            // ====================================================
            // GAMEPAD
            // ====================================================

            double driveX =
                    gamepad1.left_stick_x;

            double driveY =
                    -gamepad1.left_stick_y;

            double turn =
                    gamepad1.right_stick_x;


            // ====================================================
            // DEADZONE
            // ====================================================

            if (Math.abs(driveX)
                    < DRIVE_DEADZONE) {

                driveX = 0.0;
            }

            if (Math.abs(driveY)
                    < DRIVE_DEADZONE) {

                driveY = 0.0;
            }

            if (Math.abs(turn)
                    < DRIVE_DEADZONE) {

                turn = 0.0;
            }


            // ====================================================
            // DRIVER ACTIVE?
            // ====================================================

            boolean driverActive =
                    Math.abs(driveX) > 0.0
                    || Math.abs(driveY) > 0.0
                    || Math.abs(turn) > 0.0;


            if (driverActive) {

                driverLastActiveTime =
                        System.currentTimeMillis();

                holdActive = false;

                /*
                 * Пока водитель управляет,
                 * PID position не копит integral.
                 */

                xIntegral = 0.0;
                yIntegral = 0.0;

                headingIntegral = 0.0;

                /*
                 * Если водитель крутит робота,
                 * текущий heading становится новой целью.
                 */

                if (Math.abs(turn)
                        > DRIVE_DEADZONE) {

                    targetHeading =
                            currentHeading;

                    headingLastError =
                            0.0;
                }
            }


            // ====================================================
            // ACTIVATE HOLD
            // ====================================================

            if (!driverActive
                    && !holdActive) {

                long inactiveTime =
                        System.currentTimeMillis()
                        - driverLastActiveTime;

                if (inactiveTime
                        >= HOLD_DELAY * 1000.0) {

                    /*
                     * Фиксируем именно ТОЧКУ,
                     * где водитель отпустил управление.
                     */

                    targetX =
                            currentX;

                    targetY =
                            currentY;

                    targetHeading =
                            currentHeading;

                    resetControllers();

                    holdActive = true;
                }
            }


            // ====================================================
            // POSITION CORRECTION
            // ====================================================

            double correctionX = 0.0;
            double correctionY = 0.0;


            if (holdActive) {

                // =================================================
                // X ERROR
                // =================================================

                double xError =
                        targetX - currentX;


                // =================================================
                // X INTEGRAL
                // =================================================

                xIntegral +=
                        xError * dt;

                xIntegral =
                        clip(
                                xIntegral,
                                -1.0,
                                1.0
                        );


                // =================================================
                // X DERIVATIVE
                // =================================================

                double xDerivative =
                        (xError - xLastError)
                        / dt;

                xLastError =
                        xError;


                // =================================================
                // X PID
                // =================================================

                correctionX =
                        X_KP * xError
                        + X_KI * xIntegral
                        + X_KD * xDerivative;


                // =================================================
                // Y ERROR
                // =================================================

                double yError =
                        targetY - currentY;


                yIntegral +=
                        yError * dt;

                yIntegral =
                        clip(
                                yIntegral,
                                -1.0,
                                1.0
                        );


                double yDerivative =
                        (yError - yLastError)
                        / dt;

                yLastError =
                        yError;


                correctionY =
                        Y_KP * yError
                        + Y_KI * yIntegral
                        + Y_KD * yDerivative;


                // =================================================
                // LIMIT
                // =================================================

                correctionX =
                        clip(
                                correctionX,
                                -POSITION_MAX,
                                POSITION_MAX
                        );

                correctionY =
                        clip(
                                correctionY,
                                -POSITION_MAX,
                                POSITION_MAX
                        );
            }


            // ====================================================
            // HEADING CONTROLLER
            // ====================================================

            double rotationPower;


            if (Math.abs(turn)
                    > DRIVE_DEADZONE) {

                /*
                 * Полностью ручное вращение.
                 */

                rotationPower =
                        turn;

            } else {

                // =================================================
                // HEADING ERROR
                // =================================================

                double headingError =
                        angleWrap(
                                targetHeading
                                - currentHeading
                        );


                // =================================================
                // TOLERANCE
                // =================================================

                if (Math.abs(
                        Math.toDegrees(
                                headingError
                        )
                ) < HEADING_TOLERANCE) {

                    headingError =
                            0.0;
                }


                // =================================================
                // INTEGRAL
                // =================================================

                headingIntegral +=
                        headingError * dt;

                headingIntegral =
                        clip(
                                headingIntegral,
                                -0.25,
                                0.25
                        );


                // =================================================
                // DERIVATIVE
                // =================================================

                double derivative =
                        (
                                headingError
                                - headingLastError
                        )
                        / dt;

                headingLastError =
                        headingError;


                // =================================================
                // PID
                // =================================================

                rotationPower =
                        HEADING_KP
                        * headingError

                        + HEADING_KI
                        * headingIntegral

                        + HEADING_KD
                        * derivative;


                // =================================================
                // SIGN
                // =================================================

                rotationPower *=
                        HEADING_SIGN;


                // =================================================
                // LIMIT
                // =================================================

                rotationPower =
                        clip(
                                rotationPower,
                                -HEADING_MAX,
                                HEADING_MAX
                        );
            }


            // ====================================================
            // FIELD CENTRIC
            // ====================================================

            double fieldX =
                    driveX + correctionX;

            double fieldY =
                    driveY + correctionY;


            // ====================================================
            // LIMIT TRANSLATION
            // ====================================================

            double translationMagnitude =
                    Math.hypot(
                            fieldX,
                            fieldY
                    );


            if (translationMagnitude > 1.0) {

                fieldX /=
                        translationMagnitude;

                fieldY /=
                        translationMagnitude;
            }


            // ====================================================
            // FIELD -> ROBOT
            // ====================================================

            double cos =
                    Math.cos(
                            -currentHeading
                    );

            double sin =
                    Math.sin(
                            -currentHeading
                    );


            double robotX =
                    fieldX * cos
                    - fieldY * sin;

            double robotY =
                    fieldX * sin
                    + fieldY * cos;


            // ====================================================
            // MECANUM
            // ====================================================

            double denominator =
                    Math.max(
                            Math.abs(robotY)
                            + Math.abs(robotX)
                            + Math.abs(rotationPower),
                            1.0
                    );


            double fl =
                    (
                            robotY
                            + robotX
                            + rotationPower
                    )
                    / denominator;


            double rl =
                    (
                            robotY
                            - robotX
                            + rotationPower
                    )
                    / denominator;


            double fr =
                    (
                            robotY
                            - robotX
                            - rotationPower
                    )
                    / denominator;


            double rr =
                    (
                            robotY
                            + robotX
                            - rotationPower
                    )
                    / denominator;


            // ====================================================
            // POWER
            // ====================================================

            FL.setPower(fl);
            FR.setPower(fr);
            RL.setPower(rl);
            RR.setPower(rr);


            // ====================================================
            // TELEMETRY
            // ====================================================

            telemetry.addLine(
                    "=============================="
            );

            telemetry.addLine(
                    "       PEDRO STYLE HOLD"
            );

            telemetry.addLine(
                    "=============================="
            );

            telemetry.addData(
                    "X",
                    "%.2f",
                    currentX
            );

            telemetry.addData(
                    "Y",
                    "%.2f",
                    currentY
            );

            telemetry.addData(
                    "Heading",
                    "%.2f",
                    Math.toDegrees(
                            currentHeading
                    )
            );

            telemetry.addLine("");

            telemetry.addData(
                    "Target X",
                    "%.2f",
                    targetX
            );

            telemetry.addData(
                    "Target Y",
                    "%.2f",
                    targetY
            );

            telemetry.addData(
                    "Target Heading",
                    "%.2f",
                    Math.toDegrees(
                            targetHeading
                    )
            );

            telemetry.addLine("");

            telemetry.addData(
                    "X Error",
                    "%.2f",
                    targetX - currentX
            );

            telemetry.addData(
                    "Y Error",
                    "%.2f",
                    targetY - currentY
            );

            telemetry.addData(
                    "Heading Error",
                    "%.2f",
                    Math.toDegrees(
                            angleWrap(
                                    targetHeading
                                    - currentHeading
                            )
                    )
            );

            telemetry.addLine("");

            telemetry.addData(
                    "Hold",
                    holdActive
                            ? "ACTIVE"
                            : "DRIVER"
            );

            telemetry.addData(
                    "Rotation",
                    "%.3f",
                    rotationPower
            );

            telemetry.addData(
                    "X Correction",
                    "%.3f",
                    correctionX
            );

            telemetry.addData(
                    "Y Correction",
                    "%.3f",
                    correctionY
            );

            telemetry.addData(
                    "Pinpoint",
                    pinpoint.getDeviceStatus()
            );

            telemetry.update();
        }
    }


    // ============================================================
    // RESET CONTROLLERS
    // ============================================================

    private void resetControllers() {

        headingIntegral = 0.0;
        headingLastError = 0.0;

        xIntegral = 0.0;
        xLastError = 0.0;

        yIntegral = 0.0;
        yLastError = 0.0;

        lastTime =
                System.nanoTime();
    }


    // ============================================================
    // ANGLE WRAP
    // ============================================================

    private double angleWrap(
            double angle) {

        while (angle > Math.PI) {

            angle -=
                    2.0 * Math.PI;
        }

        while (angle < -Math.PI) {

            angle +=
                    2.0 * Math.PI;
        }

        return angle;
    }


    // ============================================================
    // CLIP
    // ============================================================

    private double clip(
            double value,
            double min,
            double max) {

        return Math.max(
                min,
                Math.min(
                        max,
                        value
                )
        );
    }
}
