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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous

public class AutoNew extends LinearOpMode {

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

        /*
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);

        }
      
         */

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory splineToA = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(5, 20), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(15, 20, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(45, 20, Math.toRadians(0)))
                .build();

        Trajectory splineToB = drive.trajectoryBuilder(splineToA.end())
                .lineToSplineHeading(new Pose2d(60.5, 14.5, Math.toRadians(-38)))
                //55 12
                .build();

        Trajectory splineToC = drive.trajectoryBuilder(splineToB.end())
                .lineToSplineHeading(new Pose2d(50.5, 16, Math.toRadians(-90)))
                //47.5
                .build();

        Trajectory splineToD = drive.trajectoryBuilder(splineToC.end())
                .lineToLinearHeading(new Pose2d(54, -29, Math.toRadians(-90)))
                .build();

        Trajectory splineToE = drive.trajectoryBuilder(splineToD.end())
                .lineToLinearHeading(new Pose2d(50.5, 0, Math.toRadians(35)))
                .build();

        Trajectory splineToF = drive.trajectoryBuilder(splineToE.end())
                .forward(15)
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
                .strafeRight(25)
                .build();

        Trajectory parkAt3 = drive.trajectoryBuilder(parkAt2.end())
                .strafeRight(-29)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            drive.followTrajectory(splineToA);

            raiseSlideHighJunk();
            sleep(2000);
            drive.followTrajectory(splineToB);

            openClaw2();
           // sleep(10000);
            drive.followTrajectory(splineToC);

            lowerSlideStack5();

            drive.followTrajectory(splineToD);
            closeClaw();
            sleep(1000);
            raiseSlideHighJunk();
            sleep(50000);

            drive.followTrajectory(splineToE);

            drive.followTrajectory(splineToF);
            //sleep(3000);
            //openClaw();
            //lowerSlideStack4();
            sleep(500);
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
        slideLeft.setTargetPosition(3050);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(3000);
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
        sleep(1000);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(5);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(5);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(0.9);
        slideRight.setPower(0.9);
    }

    public void lowerSlideStack5() {
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(600);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(600);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(0.9);
        slideRight.setPower(0.9);
    }

    public void lowerSlideStack4() {
        sleep(500);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(550);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(550);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(0.9);
        slideRight.setPower(0.9);
    }

    public void closeClaw() {
        leftClaw.setPosition(0.63);
        rightClaw.setPosition(0.39);
    }

    public void openClaw() {
        leftClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition(0.55);
        rightClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setPosition(0.43);
    }

    public void openClaw2() {
        leftClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition(0.4);
        rightClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setPosition(0.6);
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

        closeClaw();

        sleep(1000);


        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setTargetPosition(500);

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setTargetPosition(450);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRight.setPower(0.8);
        slideLeft.setPower(0.8);


    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 640;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        //tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }

    public int parkPos() {
        if (tfod == null)
            telemetry.addData("tfod is not active  ", "");
        else
            telemetry.addData("tfod is  active  ", "");
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        int parkPos = 0;

        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;

            for (Recognition recognition : updatedRecognitions) {

                if (recognition.getLabel() == "orange") {
                    parkPos = 1;
                    //A();
                }

                if (recognition.getLabel() == "red") {
                    parkPos = 2;
                    //B();
                }

                if (recognition.getLabel() == "green") {
                    parkPos = 3;
                    //C();
                }
            }
        }
        return parkPos;
    }
}




