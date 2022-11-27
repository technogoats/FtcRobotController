package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    public final double turnSpeed = 0.5;

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
                //forward
                moveForward();

            } else if (gamepad1.dpad_down) {
                //backward
                moveBackward();

            } else if (gamepad1.dpad_right) {
                //right strafe
                strafeRight();

            } else if (gamepad1.dpad_left) {
                //left strafe
                strafeLeft();

            } else if (gamepad1.right_bumper) {
                //right turn
                telemetry.addData("right bumper pressed", "");
                raiseSlideHighJunk();

            } else if (gamepad1.left_bumper) {
                //left turn
                telemetry.addData("left bumper pressed", "");
                closeClaw();
                lowerSlide();

            } else if (gamepad1.right_trigger > 0) {
                closeClaw();

            } else if (gamepad1.left_trigger > 0) {
                openClaw();

            } else if (gamepad1.left_stick_button) {

            } else if (gamepad1.right_stick_button) {

            } else if (gamepad1.x) {

            } else if (gamepad1.b) {
                openClawCap();

            } else if (gamepad1.a) {
                //small junction

            } else if (gamepad1.y) {
                //medium junction

            } else if (gamepad2.x) {

            } else if (gamepad2.y) {

            } else if (gamepad2.a) {

            } else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFR.setPower(0);
                motorFL.setPower(0);
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
        slideRight.setTargetPosition(3050);
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
        slideRight.setTargetPosition(150);
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

 /*
    public void RS3() {
        //linearSlide.setTargetPosition(500);
        // int i = 50;
        //for (i = 50; i < 1000; i += 100) {
        Basket.setPosition(0.8);
        sleep(500);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setTargetPosition(1750);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.9);
    }

    public void RS1() {
        //linearSlide.setTargetPosition(500);

        // int i = 50;
        //for (i = 50; i < 1000; i += 100) {
        Basket.setPosition(0.8);
        sleep(500);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        // 600 with 1 inch distance works
        linearSlide.setTargetPosition(600);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.9);

    }

    public void RS2() {
        //linearSlide.setTargetPosition(500);

        // int i = 50;
        //for (i = 50; i < 1000; i += 100) {
        Basket.setPosition(0.8);
        sleep(500);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setTargetPosition(900);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.9);
    }

    public void lowerSlide() {
        Basket.setPosition(0.8);
        sleep(1000);
        linearSlide.setTargetPosition(20);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.9);
        sleep(1000);
        linearSlide.setTargetPosition(0);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.9);
    }

    public void raiseBasket() {
        // Basket.setDirection(Servo.Direction.FORWARD);
        Basket.setPosition(0);
    }

    public void lowerBasket() {
        // Basket.setDirection(Servo.Direction.FORWARD);
        // Basket.setPosition(0.2);
        // Basket.setDirection(Servo.Direction.REVERSE);
        Basket.setPosition(0.8);
    }

    public void lockBasket() {
        Basket.setPosition(0.8);
        sleep(500);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setTargetPosition(100);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.9);
    }


  */

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

}