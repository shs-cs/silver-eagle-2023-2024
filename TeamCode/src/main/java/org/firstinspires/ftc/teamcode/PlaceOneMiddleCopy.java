package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "PaceOneMiddleCopy", group = "Autonomous")
public class PlaceOneMiddleCopy extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear, WormMotor;
    private Servo GripperServo;
    private double tileLength = 24.0; // Assuming each tile is 24 inches
    //private double armPosition = 24.0;

    public static final double GRIPPER_CLOSED_POSITION = 0.93;
    public static final double GRIPPER_OPENED_POSITION = 0.80;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        leftRear = hardwareMap.dcMotor.get("LeftRear");
        rightRear = hardwareMap.dcMotor.get("RightRear");
        GripperServo = hardwareMap.servo.get("GripperServo");
        //WormMotor = hardwareMap.dcMotor.get("WormMotor");

        // Reverse the motors if needed based on your robot configuration
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        GripperServo.setPosition(GRIPPER_CLOSED_POSITION);


        // Wait for the start button to be pressed
        waitForStart();


        // Autonomous code to strafe for 3 tiles
        //sleep(10000);

        moveTiles(-0.78);
        sleep(400);
        moveTiles(0.70);

        sleep(400);

        strafeTiles(1.8);




        //strafeTiles(-1.8);

        // Stop the robot
        stopRobot();
    }



    private void strafeTiles(double tiles) {
        // Calculate the target position for strafing
        int targetPosition = (int) (tiles * tileLength * countsPerInch());

        // Set target positions for each motor
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - targetPosition);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - targetPosition);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + targetPosition);

        // Set mode to RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to motors
        double power = 0.7; // Adjust this value as needed
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);

        // Wait until all motors reach their target position
        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            telemetry.addData("Status", "Strafing to position...");
            telemetry.update();
        }

        // Stop motors
        stopRobot();

        // Set mode back to RUN_USING_ENCODER
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveTiles(double tiles) {
        // Calculate the target position for strafing
        int targetPosition = (int) (tiles * tileLength * countsPerInch());

        // Set target positions for each motor
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPosition);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + targetPosition);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + targetPosition);

        // Set mode to RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to motors
        double power = 0.5; // Adjust this value as needed
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);

        // Wait until all motors reach their target position
        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            telemetry.addData("Status", "Moving to position...");
            telemetry.update();
        }

        // Stop motors
        stopRobot();

        // Set mode back to RUN_USING_ENCODER
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    private void moveArm(double armPosition) {
        // Calculate the target position for strafing
        int targetPosition = (int) (arm * tileLength * countsPerInch());

        // Set target positions for each motor
       WormMotor.setTargetPosition(WormMotor.getCurrentPosition() + targetPosition);


        // Set mode to RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to motors
        double power = 0.5; // Adjust this value as needed
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);

        // Wait until all motors reach their target position
        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            telemetry.addData("Status", "Moving to position...");
            telemetry.update();
        }

        // Stop motors
        stopRobot();

        // Set mode back to RUN_USING_ENCODER
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    */

    private void stopRobot() {
        // Stop all motors

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        // WormMotor.setPower(0);
    }

    private double countsPerInch() {
        // Adjust this value based on your robot's configuration and encoder ticks per revolution
        return 59.0;
    }
}
