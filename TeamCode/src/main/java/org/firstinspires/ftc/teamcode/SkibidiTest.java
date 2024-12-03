package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SkibidiTest", group = "Autonomous")

public class SkibidiTest extends LinearOpMode{
    public double GrabbingPosition = 0.0;
    public double RestPosition = 0.92;
    public double SpecimenPosition = 0.32;
    public double HighBasketPosition = 0.32;
    public double WristStraight = 0.0;
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
        TwistyTurnyServo = hardwareMap.get(Servo.class, "TwistyTurnyServo");
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

//**************************** Put Initialization Code Under Here 0_0 ******************************

        TwistyTurnyServo.setPosition(WristStraight);

        clawClose();

        WristServo.setPosition(RestPosition);



        waitForStart();
//***************************** Put Auto Code Under Here To Run :o (please work) *******************

        OneBasketAttempt();


        /* MoveTiles(0.5);
        Pause();
        StrafeTilesLeft(0.5);
        Pause();
        ArmUp(1.0, 1000); // test to find the positioning for specimen scoring
        Pause();
        ViperVroomUp(0.8, 1000); // test to find the positioning for specimen scoring
        Pause();
        ArmUp(1.0,100); // test to find the positioning for specimen scoring
        Pause();
        ArmDown(1.0, 100); // test to find the positioning for specimen scoring
        Pause();
        clawOpen();
        Pause();
        MoveTilesBackwards(0.5);
    */



//*************************************** End of Auto :( (we're doomed)*****************************
    }
//************************************ Autonomous Functions :) *************************************

    public void moveForward(double power, int time){
        LeftFront.setPower(-power);
        RightFront.setPower(-power);
        LeftRear.setPower(-power);
        RightRear.setPower(-power);
        sleep(time);
    }

    public void Pause()
    {
        stopMotors();
        sleep(500);
        stopMotors();
    }


    public void moveBackward(double power, int time)
    {
        moveForward(-power, time);
    }




    public void TurnLeft(double power, int time){
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

    public void OneBasketAttempt()
    {
        StrafeTilesRight(1.2);
        Pause();
        TurnLeft(0.5,400);
        Pause();
        ArmUp(1.0,2030);
        Pause();
        ViperVroomUp(0.8,2030);
        Pause();
        MoveWrist(HighBasketPosition);
        Pause();
        MoveTiles(0.8);
        Pause();
        MoveTiles(0.1);
        Pause();
        clawOpen();
        Pause();
        MoveTilesBackwards(0.3);
        Pause();



    }

    public void TurnRight(double power, int time){
        LeftFront.setPower(-power);
        RightFront.setPower(power);
        LeftRear.setPower(-power);
        RightRear.setPower(power);
        sleep(time);
    }

    public void ArmUp(double power, int time){
        LeftArmMotor.setPower(-power);
        RightArmMotor.setPower(-power);
        sleep(time);

    }

    public void ArmDown(double power, int time)
    {


        LeftArmMotor.setPower(power);
        RightArmMotor.setPower(power);
        sleep(time);

    }


    public void ViperVroomDown(double power, int time)
    {
        ViperMotor.setPower(power);
        sleep(time);

    }

    public void ViperVroomUp(double power, int time)
    {
        ViperMotor.setPower(-power);
        sleep(time);

    }



    public void clawOpen()
    {
        LeftGripperServo.setPosition(0.4); // Close left gripper
        RightGripperServo.setPosition(0.4);
    }

    public void clawClose()
    {
        LeftGripperServo.setPosition(0.25); // Close left gripper
        RightGripperServo.setPosition(0.25);
    }

    public void MoveWrist(double pos)
    {
        WristServo.setPosition(pos); // Close left gripper

    }

    public void MoveTiles(double amount)
    {
        moveForward(0.3, (int)(1390 * amount));
        stopMotors();


    }


    public void MoveTilesBackwards(double amount)
    {
        moveBackward(0.3, (int)(1390 * amount));
        stopMotors();


    }








    public void StrafeTilesLeft(double amount)
    {

        StrafeLeft(0.3,(int)(2100 * amount));
        stopMotors();
        sleep(300);
        stopMotors();
        TurnRight(0.3,63);
        stopMotors();

    }







    public void StrafeTilesRight(double amount)
    {

        StrafeRight(0.3,(int)(2100 * amount));
        stopMotors();
        sleep(300);
        stopMotors();
        TurnLeft(0.3,63);
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
