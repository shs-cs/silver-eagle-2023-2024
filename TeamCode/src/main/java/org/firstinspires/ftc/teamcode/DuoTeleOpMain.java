package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "DuoTeleOpMain", group = "TeleOp")
public class DuoTeleOpMain extends OpMode {
    //setting up motors and servos for use + Setting up Positions
    public double TwistyTurnySidePosition = 0.3;
    public double TwistyTurnyFlipPosition = 0.65;
    public double WristGrabbingPosition = 0.0;
    public double ClawOpenNormalPos =  0.37;
    public double ClawOpenWidePos = 0.47;
    public double ClawNomNom = 0.25;
    public double WristRestPosition = 0.92;
    public double WristSlammaJammaPosition = 0.85;
    public double WristSpecimenPosition = 0.32;
    public double WristBackScoringPosition = 0.485;
    public double WristHighBasketPosition = 0.32;
    public double TwistyTurnyStraight = 0.0;
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftRear;
    private DcMotor RightRear;
    private DcMotor LeftArmMotor;
    private DcMotor RightArmMotor;
    private DcMotor ViperMotor;
    private Servo LeftGripperServo;
    private Servo RightGripperServo;
    private Servo WristServo;
    private Servo TwistyTurnyServo;

    @Override
    public void init() {
        // Initializing hardware to driver hub
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        LeftRear = hardwareMap.dcMotor.get("LeftRear");
        RightRear = hardwareMap.dcMotor.get("RightRear");
        ViperMotor = hardwareMap.dcMotor.get("ViperMotor");
        LeftGripperServo = hardwareMap.get(Servo.class, "LeftGripperServo");
        WristServo = hardwareMap.get(Servo.class, "WristServo");
        RightGripperServo = hardwareMap.get(Servo.class, "RightGripperServo");
        TwistyTurnyServo = hardwareMap.get(Servo.class, "TwistyTurnyServo");
        RightGripperServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);

        LeftArmMotor = hardwareMap.dcMotor.get("LeftArmMotor");
        RightArmMotor = hardwareMap.dcMotor.get("RightArmMotor");
        LeftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the direction of the correct motors
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);

        // Set the motors to brake when power is set to 0
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop()
    {
        handleDriveTrain();
        ArmGoVroom();
        ViperSlideVroom();
        ServoGoVroom();
    }

    public void handleDriveTrain()
    {
        double leftStickY = gamepad1.left_stick_y; // Y axis for forward/backward
        double leftStickX = gamepad1.left_stick_x; // Left stick X for strafe
        double rightStickX = gamepad1.right_stick_x; // Right stick X for rotation

        //Rename Stick variables For better understanding
        double strafePower = leftStickX; // Lateral movement
        double forwardPower = leftStickY * 0.8; // Forward/backward movement
        double rotationPower = rightStickX * 0.5; // Rotation

        // Calculate power for each motor
        double powerLF = getPower(forwardPower - strafePower - rotationPower);  // Left Front
        double powerRF = getPower(forwardPower + strafePower + rotationPower);  // Right Front
        double powerLR = getPower(forwardPower + strafePower - rotationPower);  // Left Rear
        double powerRR = getPower(forwardPower - strafePower + rotationPower);  // Right Rear

        // Set motor powers
        LeftFront.setPower(powerLF);
        RightFront.setPower(powerRF);
        LeftRear.setPower(powerLR);
        RightRear.setPower(powerRR);

        telemetry.addData("RightFront Power:", RightFront.getPower());
        telemetry.addData("LeftFront Power:", LeftFront.getPower());
        telemetry.addData("RightRear Power:", RightRear.getPower());
        telemetry.addData("LeftRear Power:", LeftRear.getPower());
        telemetry.update();



    }


    public void ViperSlideVroom() {

        if (gamepad2.right_stick_y > 0.6) {
            ViperMotor.setPower(0.8);
        }

        if (gamepad2.right_stick_y < -0.6) {
            ViperMotor.setPower(-0.8);
        }

        if (gamepad2.right_stick_y == 0) {
            ViperMotor.setPower(0);
        }


        telemetry.addData("ViperMotor Power", ViperMotor.getPower());
        telemetry.update();
    }

    public void ServoGoVroom() {


        if (gamepad2.left_bumper) {

            RightGripperServo.setPosition(ClawOpenNormalPos); // Open Right Gripper
            LeftGripperServo.setPosition(ClawOpenNormalPos); // Open Left Gripper
            telemetry.addData("Right Gripper Position:", RightGripperServo.getPosition());
            telemetry.addData("Left Gripper Position:", LeftGripperServo.getPosition());
            telemetry.update();
        }


        if (gamepad2.right_bumper)
        {
            LeftGripperServo.setPosition(ClawNomNom); // Close Left Gripper
            RightGripperServo.setPosition(ClawNomNom); // Close Right Gripper
            telemetry.addData("Right Gripper Position:", RightGripperServo.getPosition());
            telemetry.addData("Left Gripper Position:", LeftGripperServo.getPosition());
            telemetry.update();
        }

        if(gamepad2.dpad_left)
        {
            TwistyTurnyServo.setPosition(TwistyTurnySidePosition);
            telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            telemetry.update();
        }

        if(gamepad2.dpad_right)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyStraight);
            telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            telemetry.update();
        }


        if(gamepad1.a)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyStraight);
            telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            telemetry.update();
        }


        if(gamepad1.b)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyFlipPosition);
            telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            telemetry.update();
        }





        if(gamepad2.dpad_up)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyFlipPosition);
            telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            telemetry.update();
        }


        if (gamepad2.x)
        {
            WristServo.setPosition(WristGrabbingPosition);
            telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            telemetry.update();
        }

        if (gamepad2.y)
        {
            WristServo.setPosition(WristHighBasketPosition);
            telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            telemetry.update();
        }

        if (gamepad2.b && gamepad2.dpad_down)
        {
            WristServo.setPosition(WristRestPosition);
            telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            telemetry.update();
        }

        if (gamepad2.a)
        {
            WristServo.setPosition(WristBackScoringPosition);
            telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            telemetry.update();
        }


        if (gamepad2.left_trigger > 0.5)
        {
           LeftGripperServo.setPosition(ClawOpenWidePos);
           RightGripperServo.setPosition(ClawOpenWidePos);
           telemetry.addData("Right Gripper Position:", RightGripperServo.getPosition());
           telemetry.addData("Left Gripper Position:", LeftGripperServo.getPosition());
           telemetry.update();
        }



        if (gamepad2.right_trigger > 0.5)
        {

            WristServo.setPosition(WristSlammaJammaPosition);

        }
    }

    public void ArmGoVroom(){

        if (gamepad2.left_stick_y < -0.5)
        {
            LeftArmMotor.setPower(1.0);
            RightArmMotor.setPower(1.0);

            telemetry.addData("LeftArmMotor Power:", LeftArmMotor.getPower());
            telemetry.addData("RightArmMotor Power:", RightArmMotor.getPower());
            telemetry.update();

        }



        if (gamepad2.left_stick_y > 0.5)
        {
            LeftArmMotor.setPower(-0.7);
            RightArmMotor.setPower(-0.7);

            telemetry.addData("LeftArmMotor Power:", LeftArmMotor.getPower());
            telemetry.addData("RightArmMotor Power:", RightArmMotor.getPower());
            telemetry.update();
        }

        if (gamepad2.left_stick_y == 0) {
            LeftArmMotor.setPower(0);
            RightArmMotor.setPower(0);
        }



    }

    public double getPower(double powerLevel) {

        if (powerLevel > 1) {
            return 1;
        }

        else if (powerLevel < -1) {
            return -1;
        }

        return powerLevel;
    }

    @Override
    public void stop() {
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);
        RightRear.setPower(0);
        LeftArmMotor.setPower(0);
        RightArmMotor.setPower(0);
        ViperMotor.setPower(0);
    }
}