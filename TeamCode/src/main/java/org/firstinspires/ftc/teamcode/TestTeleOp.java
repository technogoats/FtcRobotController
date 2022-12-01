package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import  java.lang.Math;

@TeleOp

public class TestTeleOp extends LinearOpMode {

    //Wheel motors
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;

    //Grabber
    //private Servo Arm;
    //private Servo Hand;
    //private CRServo Ducky;

    //private Servo Basket;

    private DcMotor slideLeft;
    private DcMotor slideRight;

    //Odometer
    private DcMotor verticalLeft; // BL
    private DcMotor verticalRight; // FR
    private DcMotor horizontal; // BR

    private Servo leftClaw;
    private Servo rightClaw;

    //Wheel stuff
    public final double wheelPower = -0.5;
    public final double turnSpeed = 0.7;

    double   leftX = 0, leftY = 0, rightX = 0, V = 0, W = 0, Right = 0, Left = 0;
    int rightSlideHeight, leftSlideHeight;


    public double power = 0.7;

    private Blinker expansion_Hub_3;

    public void runOpMode() {
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        //Arm = hardwareMap.get(Servo.class, "Arm");
        //Hand = hardwareMap.get(Servo.class, "Hand");
        //Ducky = hardwareMap.get(CRServo.class, "Ducky");
        // Ducky = hardwareMap.crservo.get("Ducky");
        //intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        //Basket = hardwareMap.get(Servo.class, "Basket");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        //Ducky = hardwareMap.get(CRServo.class, "Ducky");
        verticalLeft = hardwareMap.get(DcMotor.class, "motorBL");
        verticalRight = hardwareMap.get(DcMotor.class, "motorFR");
        horizontal = hardwareMap.get(DcMotor.class, "motorBR");

        expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");

        initDriveTrain();

        waitForStart();
        telemetry.addData("you can", "start now");
        telemetry.update();

        while (opModeIsActive()) {

            //showOdo();
            //telemetry.addData("position", linearSlide.getCurrentPosition());
            // telemetry.update();
            //intakeMotor.setPower(0.8);

            if (gamepad1.dpad_up) {

            } else if (gamepad1.dpad_down) {


            } else if (gamepad1.dpad_right) {

            } else if (gamepad1.dpad_left) {

            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                //right turn
                telemetry.addData("right bumper pressed", "");
                raiseSlideHighJunk();

            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                //left turn
                telemetry.addData("left bumper pressed", "");
                closeClaw();
                raiseSlideCone();

            } else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0 ) {
                closeClaw();

            } else if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
                openClaw();

            } else if (gamepad1.left_stick_button) {

            } else if (gamepad1.right_stick_button) {

            } else if (gamepad1.x || gamepad2.x) {
                openClawCap();

            } else if (gamepad1.b || gamepad2.b) {
                lowerSlide();

            } else if (gamepad1.a || gamepad2.a) {
                //small junction
                raiseSlideLowJunk();

            } else if (gamepad1.y || gamepad2.y) {
                //medium junction
                raiseSlideMediumJunk();

            } else if (gamepad2.right_trigger > 0) {
                leftSlideHeight = slideLeft.getCurrentPosition();
                rightSlideHeight = slideRight.getCurrentPosition();

                if (leftSlideHeight < 3000 && rightSlideHeight <3000) {
                    slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                    slideLeft.setTargetPosition(leftSlideHeight + 100);

                    slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
                    slideRight.setTargetPosition(rightSlideHeight + 100);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    slideRight.setPower(0.9);
                    slideLeft.setPower(0.9);
                }

            }
            else if (gamepad2.left_trigger > 0) {
                /*
                leftSlideHeight = slideLeft.getCurrentPosition();
                rightSlideHeight = slideRight.getCurrentPosition();

                if (leftSlideHeight > 100 && rightSlideHeight > 100) {
                    slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                    slideLeft.setTargetPosition(leftSlideHeight - 100);

                    slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
                    slideRight.setTargetPosition(rightSlideHeight - 100);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    slideRight.setPower(0.7);
                    slideLeft.setPower(0.7);

                   // sleep(200);
                }

                 */

            } else if (gamepad2.y) {

            } else if (gamepad2.a) {

            }
            else if (gamepad1.left_stick_y != 0) {

                leftY = gamepad1.left_stick_y;

                //telemetry.addData("Mode", "running");
                //telemetry.addData("sticks", "  left x=" + gamepad1.left_stick_x + "  right y=" + gamepad1.left_stick_y);
                //telemetry.update();

                resetMotorDirection();

                leftY = scaleInput(leftY);

                motorBL.setPower(power *(Range.clip(leftY, -1, 1)));
                motorBR.setPower(power *(Range.clip(-leftY, -1, 1)));
                motorFL.setPower(power *(Range.clip(leftY, -1, 1)));
                motorFR.setPower(power *(Range.clip(-leftY, -1, 1)));
            }

            else if (gamepad1.right_stick_x != 0) {
                rightX = gamepad1.right_stick_x;
                resetMotorDirection();

                rightX = scaleInput(rightX);

                motorBL.setPower(turnSpeed *(Range.clip(-rightX, -1, 1)));
                motorBR.setPower(turnSpeed *(Range.clip(-rightX, -1, 1)));
                motorFL.setPower(turnSpeed *(Range.clip(-rightX, -1, 1)));
                motorFR.setPower(turnSpeed *(Range.clip(-rightX, -1, 1)));


            }

            else if (gamepad1.left_stick_x != 0) {

                leftX = gamepad1.left_stick_x;

                //telemetry.addData("Mode", "running");
                //telemetry.addData("sticks", "  left x=" + gamepad1.left_stick_x + "  right y=" + gamepad1.left_stick_y);
                //telemetry.update();

                resetMotorDirection();

                leftX = scaleInput(leftX);

                motorBL.setPower(power *(Range.clip(leftX, -1, 1)));
                motorBR.setPower(power *(Range.clip(leftX, -1, 1)));
                motorFL.setPower(power *(Range.clip(-leftX, -1, 1)));
                motorFR.setPower(power *(Range.clip(-leftX, -1, 1)));
            }

            else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFR.setPower(0);
                motorFL.setPower(0);
                resetMotorDirection();
            }
        }

    }

    private void showOdo() {
        telemetry.addData("Test: ", verticalLeft.getCurrentPosition());
        telemetry.addData("verticalLeft TickCount: ", verticalLeft.getCurrentPosition());
        telemetry.addData("verticalRight TickCount: ", verticalRight.getCurrentPosition());
        telemetry.addData("horizontal TickCount: ", horizontal.getCurrentPosition());
        telemetry.update();
    }

    public void resetMotorDirection(){
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveForward() {
        motorBL.setPower(wheelPower);
        motorBR.setPower(-wheelPower);
        motorFR.setPower(-wheelPower);
        motorFL.setPower(wheelPower);
    }

    public void moveBackward() {
        motorBL.setPower(-wheelPower);
        motorBR.setPower(wheelPower);
        motorFR.setPower(wheelPower);
        motorFL.setPower(-wheelPower);
    }

    public void turnRight() {
        motorBL.setPower(-turnSpeed);
        motorBR.setPower(-turnSpeed);
        motorFR.setPower(-turnSpeed);
        motorFL.setPower(-turnSpeed);
    }

    public void turnLeft() {
        motorBL.setPower(turnSpeed);
        motorBR.setPower(turnSpeed);
        motorFR.setPower(turnSpeed);
        motorFL.setPower(turnSpeed);
    }

    public void strafeRight() {
        motorBL.setPower(-wheelPower);
        motorBR.setPower(-wheelPower);
        motorFR.setPower(wheelPower);
        motorFL.setPower(wheelPower);
    }

    public void strafeLeft() {
        motorBL.setPower(wheelPower);
        motorBR.setPower(wheelPower);
        motorFR.setPower(-wheelPower);
        motorFL.setPower(-wheelPower);
    }

    public void raiseSlideHighJunk() {
        //int i = 50;
        //for (i = 50; i < 1000; i += 100) {
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(3100);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(3100);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRight.setPower(0.9);
        slideLeft.setPower(0.9);

        //}
    }

    public void raiseSlideMediumJunk() {
        //int i = 50;
        //for (i = 50; i < 1000; i += 100) {
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(2225);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(2225);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRight.setPower(0.9);
        slideLeft.setPower(0.9);
        //}
    }


    public void raiseSlideLowJunk() {
        //int i = 50;
        //for (i = 50; i < 1000; i += 100) {
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(1435);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(1435);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRight.setPower(0.9);
        slideLeft.setPower(0.9);
        //}
    }

    public void raiseSlideCone() {
        //int i = 50;
        //for (i = 50; i < 1000; i += 100) {
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(200);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(200);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRight.setPower(0.9);
        slideLeft.setPower(0.9);
        //}
    }

    public void lowerSlide() {
        closeClaw();
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(5);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(5);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(0.9);
        slideRight.setPower(0.9);
    }

    public void closeClaw(){
        leftClaw.setPosition(0.62);
        rightClaw.setPosition(0.4);
    }

    public void openClaw(){
        leftClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition(0.55);
        rightClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setPosition(0.43);
    }

    public void openClawCap(){
        leftClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition(0.52);
        rightClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setPosition(0.46);
    }

    private void initDriveTrain() {

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }

}

