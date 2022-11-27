/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous

public class AutoOld extends LinearOpMode {

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

    private Blinker expansion_Hub_3;

    public void runOpMode() {
        //initAll();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory driveToHub = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(180)))
                .build();

        Trajectory driveToFreight = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
                .build();

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(distance)
                .build();

        Trajectory backward = drive.trajectoryBuilder(new Pose2d())
                .forward(distance)
                .build();

        Trajectory driveToHub2 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)))
                .build();

        Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .forward(distance)
                .build();

        waitForStart();

        while(opModeIsActive()) {
           /*
            //detect
            //1,2,3
            drive.followTrajectory(driveToHub); // linetosplineheading(endtopose3d) front, turn 180
           // RS3(); //raise slide
            //raiseBasket();//drop
            lowerSlide(); //lowerslide
            drive.followTrajectory(driveToFreight); //moveforward //straferight //turn right90
            //startIntake(); //swallow block
            distance = 15;
            drive.followTrajectory(forward);
           // lockBasket();
            drive.followTrajectory(backward);
            drive.followTrajectory(driveToHub2);//  linetosplineheading(endtopose3d) back, turn 90
            distance = 40;
            drive.followTrajectory(backward); // back
            //RS3(); //raise slide
           // raiseBasket();//drop
            lowerSlide(); //lowerslide
            distance = 40;
            drive.followTrajectory(left); //strafe left
            //park





            // go to cones, spline to linear heading; raise to 5, open, close
            //line to linear heading to high juntion; raise slide to high junct, release
            // line to lin heading in rveerse; lower slide to 4







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

  */
