package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SkibidiTest", group = "Autonomous")

public class SkibidiTest extends LinearOpMode{

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
    private Servo TwistyTurny;

    @Override
    public void runOpMode() {
        // Initializing hardware
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        LeftRear = hardwareMap.dcMotor.get("LeftRear");
        RightRear = hardwareMap.dcMotor.get("RightRear");
        ViperMotor = hardwareMap.dcMotor.get("ViperMotor");
        LeftGripperServo = hardwareMap.get(Servo.class, "LeftGripperServo");
        WristServo = hardwareMap.get(Servo.class, "WristServo");
        RightGripperServo = hardwareMap.get(Servo.class, "RightGripperServo");
        RightGripperServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);

        LeftArmMotor = hardwareMap.dcMotor.get("LeftArmMotor");
        RightArmMotor = hardwareMap.dcMotor.get("RightArmMotor");
        LeftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the direction of the right motors
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);

        // Set the motors to brake when power is set to 0
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawClose();

        waitForStart();

        // *** Autonomous Sequence for opening/closing claw to get one block ***


        // *** Autonomous Sequence for moving to basket ***
        //MoveTiles(0.5);

        turnLeft(0.5,870);

       /* turnLeft(0.5,980);

        moveForward(0.3,1750);
        stopMotors();
        armUp(1.0,1930);
        stopMotors();
        ViperVroomUp(-0.7,1970);
        stopMotors();
        Strafeleft(0.5,270);
        stopMotors();
        turnLeft(0.5,50);
        stopMotors();

        MoveWrist(0.15);
        stopMotors();
        moveForward(0.3,800);
        stopMotors();
        Strafeleft(0.5,40);
        stopMotors();
        turnLeft(0.5,120);
        stopMotors();
        moveForward(0.3,100);

        stopMotors();


        clawOpen();


        */



        // moveBackward(0.5, 2000);
        //turnRight(0.5, 1000);
        //turnLeft(0.5, 1000);

        // *** Autonomous Sequence for raising arm completely (for top basket) ***

        // *** Autonomous Sequence for opening/closing claw to drop in basket ***

        // *** Autonomous Sequence to park robot ***

        // *** Stop Motors when parked***

        clawClose();



        stopMotors();


    }

    // *** Autonomous Functions ***

    public void moveForward(double power, int time){
        LeftFront.setPower(-power);
        RightFront.setPower(-power);
        LeftRear.setPower(-power);
        RightRear.setPower(-power);
        sleep(time);
    }

    public void moveBackward(double power, int time){
        moveForward(-power, time);
    }

    public void turnLeft(double power, int time){
        LeftFront.setPower(power);
        RightFront.setPower(-power);
        LeftRear.setPower(power);
        RightRear.setPower(-power);
        sleep(time);
    }

    public void StrafeLeft(double power, int time){
        LeftFront.setPower(power);
        RightFront.setPower(-power);
        LeftRear.setPower(-power);
        RightRear.setPower(power);
        sleep(time);
    }



    public void StrafeRight(double power, int time){
        LeftFront.setPower(-power);
        RightFront.setPower(power);
        LeftRear.setPower(power);
        RightRear.setPower(-power);
        sleep(time);
    }



    public void turnRight(double power, int time){
        LeftFront.setPower(-power);
        RightFront.setPower(power);
        LeftRear.setPower(-power);
        RightRear.setPower(power);
        sleep(time);
    }

    public void armUp(double power, int time){
        LeftArmMotor.setPower(-power);
        RightArmMotor.setPower(-power);
        sleep(time);

    }

    public void armDown(double power, int time)
    {


        LeftArmMotor.setPower(power);
        RightArmMotor.setPower(power);
        sleep(time);

    }


    public void ViperVroomUp(double power, int time)
    {
        ViperMotor.setPower(power);
        sleep(time);

    }

    public void ViperVroomDown(double power, int time)
    {
        ViperMotor.setPower(-power);
        sleep(time);

    }



    public void clawOpen()
    {
        LeftGripperServo.setPosition(0.28); // Close left gripper
        RightGripperServo.setPosition(0.28);
    }

    public void clawClose()
    {
        LeftGripperServo.setPosition(0.0); // Close left gripper
        RightGripperServo.setPosition(0.0);
    }

    public void MoveWrist(double pos)
    {
        WristServo.setPosition(pos); // Close left gripper

    }

    public void MoveTiles(double amount)
    {
        moveForward(0.3, (int)(1390 * amount));
        stopMotors();
        sleep(300);
        turnRight(0.3,60);
        stopMotors();

    }

    public void StrafeTiles(double amount)
    {

        StrafeLeft(0.3,(int)(2100 * amount));
        stopMotors();
        sleep(300);
        turnRight(0.3,63);
        stopMotors();


    }







    public void stopMotors(){
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);
        RightRear.setPower(0);
        LeftArmMotor.setPower(0);
        RightArmMotor.setPower(0);
        ViperMotor.setPower(0);


    }
}
