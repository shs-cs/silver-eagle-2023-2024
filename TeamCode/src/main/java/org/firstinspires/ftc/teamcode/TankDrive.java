


package com.example.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tank Drive", group="TeleOp")
public class TankDrive extends OpMode {

    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor RightRear;
    private DcMotor LeftRear;


    @Override
    public void init() {
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightRear = hardwareMap.dcMotor.get("RightRear");
        LeftRear = hardwareMap.dcMotor.get("LeftRear");

        // Set motor directions
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.FORWARD);

        // LeftFront2.setDirection(DcMotor.Direction.FORWARD);
        //  RightFront2.setDirection(DcMotor.Direction.REVERSE);
        // RightRear2.setDirection(DcMotor.Direction.FORWARD);
        // LeftRear2.setDirection(DcMotor.Direction.FORWARD);




        // Set zero power behavior to brake
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // LeftFront2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // RightFront2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // RightRear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // LeftRear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop() {
        // Tank drive control
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;

        // Apply zero brake when joysticks are released.
        if (Math.abs(leftPower) < 0.05) {
            leftPower = 0;
        }
        if (Math.abs(rightPower) < 0.05) {
            rightPower = 0;
        }

        LeftFront.setPower(leftPower);
        RightFront.setPower(rightPower);
        RightRear.setPower(rightPower);
        LeftRear.setPower(leftPower);
        // LeftFront2.setPower(leftPower);
        // RightFront2.setPower(rightPower);
        // RightRear2.setPower(rightPower);
        // LeftRear2.setPower(leftPower);

        // Add telemetry if needed
        telemetry.addData("LeftFront Power", LeftFront.getPower());
        telemetry.addData("RightFront Power", RightFront.getPower());
        telemetry.addData("RightRear Power", RightRear.getPower());
        telemetry.addData("LeftRear Power", LeftRear.getPower());
        //  telemetry.addData("LeftFront Power", LeftFront2.getPower());
        // telemetry.addData("RightFront Power", RightFront2.getPower());
        // telemetry.addData("RightRear Power", RightRear2.getPower());
        // telemetry.addData("LeftRear Power", LeftRear2.getPower());
        telemetry.update();
    }
}

