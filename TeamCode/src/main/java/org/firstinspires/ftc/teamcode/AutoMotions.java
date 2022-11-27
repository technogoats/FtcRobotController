package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous

public class AutoMotions extends LinearOpMode {

    //Wheel motors
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;

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
    private int distance = 0;

    private static final String TFOD_MODEL_ASSET = "model_20221126_114134.tflite";
    private static final String[] LABELS = {
            "green",
            "orange",
            "red"
    };

    private static final String VUFORIA_KEY =
            "AUJ2ej7/////AAABmROxx/SElUj2vKbQpZUiDS0FSjZ7jKGX+mBHpGQ9o81sDzBOwGnGGLFYk3+KFlqeMOl6A+6aYyfqarVowPYzfYgqkgYbyjeMR0tiBDEYTjI4HpduTmcM3NpDtVLkekFrSXa7L4liAB5mPUhDUmDrYxKsSjYz08vDtv5bHsRFEwDZHcQChOBfH6TaYDPjdoQ4XqlTPby1xZ6OOyqVDwzvTASSaVd5/s8aKQ4caEyH9EvVMmzjlHVxnBCgLaA8bC8Y11gc/ZSIChfdSAEoFegSO64xXSbgGLfzC1MMzf+ZCnzr8ShN0qOro/Pp1dVzakpmQV5nxaMWJh8ceZUfdB/VXVlCsLIaLiz/ZUuB2kv5MmER";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private Blinker expansion_Hub_3;

    public void runOpMode() {
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        verticalLeft = hardwareMap.get(DcMotor.class, "motorBL");
        verticalRight = hardwareMap.get(DcMotor.class, "motorFR");
        horizontal = hardwareMap.get(DcMotor.class, "motorBR");

        expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        initDriveTrain();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory splineToA = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(5, 17.5), Math.toRadians(0))
                .forward(40)
                .build();

        Trajectory splineToB = drive.trajectoryBuilder(splineToA.end())
                .lineToSplineHeading(new Pose2d(57, 9, Math.toRadians(-40)))
                //55 12
                .build();

        Trajectory splineToC = drive.trajectoryBuilder(splineToB.end())
                .lineToSplineHeading(new Pose2d(47.5, 17, Math.toRadians(-90)))
                .build();

        Trajectory splineToD = drive.trajectoryBuilder(splineToC.end())
                .forward(46)
                .build();

        Trajectory splineToE = drive.trajectoryBuilder(splineToD.end())
                .lineToLinearHeading(new Pose2d(47.5, 0, Math.toRadians(34)))
                .build();

        Trajectory splineToF = drive.trajectoryBuilder(splineToE.end())
                .forward(9)
                .build();

        Trajectory splineToFG = drive.trajectoryBuilder(splineToE.end())
                .back(1)
                .build();

        Trajectory splineToG = drive.trajectoryBuilder(splineToFG.end())
                .lineToLinearHeading(new Pose2d(47, -30, Math.toRadians(-90)))
                .build();

        Trajectory splineToH = drive.trajectoryBuilder(splineToG.end())
                .lineToLinearHeading(new Pose2d(47.5, 0, Math.toRadians(34)))
                .build();

        Trajectory parkAt2 = drive.trajectoryBuilder(splineToH.end())
                .lineToLinearHeading(new Pose2d(47.5, -3, Math.toRadians(0)))
                .build();

        Trajectory parkAt1 = drive.trajectoryBuilder(parkAt2.end())
                .strafeRight(23)
                .build();

        Trajectory parkAt3 = drive.trajectoryBuilder(parkAt2.end())
                .strafeRight(-29)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                    }
                    telemetry.update();
                }
            }


            drive.followTrajectory(splineToA);
            //raiseSlideHighJunk();
            //sleep(2000);
            drive.followTrajectory(splineToB);
            drive.followTrajectory(splineToC);
            drive.followTrajectory(splineToD);
            drive.followTrajectory(splineToE);
            drive.followTrajectory(splineToF);
            drive.followTrajectory(splineToFG);
            drive.followTrajectory(splineToG);
            drive.followTrajectory(splineToH);
            drive.followTrajectory(parkAt2);
            drive.followTrajectory(parkAt1);
            sleep(30000);

            //break;

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

        slideRight.setPower(0.8);
        slideLeft.setPower(0.8);
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

        slideRight.setPower(0.8);
        slideLeft.setPower(0.8);
        //}
    }

    public void lowerSlide() {
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(5);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(5);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(0.8);
        slideRight.setPower(0.8);
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

        closeClaw();

        /*
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(500);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(450);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRight.setPower(0.8);
        slideLeft.setPower(0.8);

         */

    }

}