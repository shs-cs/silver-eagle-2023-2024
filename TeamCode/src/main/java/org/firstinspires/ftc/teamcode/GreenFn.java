package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "GreenFn", group = "Autonomous")

public class GreenFn extends LinearOpMode{
    //setting up motors and servos for use + Setting up Positions 
    public int ArmHighBasketPosition = -19;
    public int ArmSpecimenPosition = -17;
    public double WristGrabbingPosition = 0.0;
    public double ClawOpenNormalPosition =  0.37;
    public double ClawOpenWidePosition = 0.4;
    public double ClawNomNom = 0.25;
    public double WristRestPosition = 0.92;
    public double WristSpecimenPosition = 0.32;
    public double WristHighBasketPosition = 0.32;
    public double TwistyTurnyWristStraight = 0.675;
    public double TwistyTurnyFlipPosition = 0.0;
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

    private static final int ticks = 1120;
    private static final double gear_ratio = 1.0;
    private static final double countsPerInch = ((ticks * gear_ratio )/(Math.PI *1.5));


    @Override
    public void runOpMode() {
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
        LeftArmMotor = hardwareMap.dcMotor.get("LeftArmMotor");
        RightArmMotor = hardwareMap.dcMotor.get("RightArmMotor");


        // Reverse the direction of the correct motors and servos
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightGripperServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);

        // Set the motors to brake when power is set to 0
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//**************************** Put Initialization Code Under Here 0_0 ******************************

        TwistyTurnyServo.setPosition(TwistyTurnyWristStraight);
        clawClose();
        WristServo.setPosition(WristRestPosition);


        /*LeftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         */


        waitForStart();
//***************************** Put Auto Code Under Here To Run :o (please work) *******************
       // SetArmPosition(1,ArmSpecimenPosition);
       // MoveForwardPosition(0.6, -5);
        //Pause(500);


        OneBasketAttempt();



//     Specimen scoring auto (WORK IN PROGRESS)
        /* MoveTiles(0.5);
        Pause(500);
        StrafeTilesLeft(0.5);
        Pause(500);
        ArmUp(1.0, 1000); // test to find the positioning for specimen scoring
        Pause(500);
        ViperVroomUp(0.8, 1000); // test to find the positioning for specimen scoring
        Pause(500);
        ArmUp(1.0,100); // test to find the positioning for specimen scoring
        Pause(500);
        ArmDown(1.0, 100); // test to find the positioning for specimen scoring
        Pause(500);
        clawOpen();
        Pause(500);
        MoveTilesBackwards(0.5);
    */



//********************************** End of Auto :( (we're doomed) *********************************
    }
//************************************ Autonomous Functions :) *************************************

    public void moveForward(double power, int time){
        LeftFront.setPower(-power);
        RightFront.setPower(-power);
        LeftRear.setPower(-power);
        RightRear.setPower(-power);
        sleep(time);
    }

   //cooler version of sleep :0 (holy moly)
    public void Pause(int MilliSeconds)
    {
        stopMotors();
        sleep(MilliSeconds);
        stopMotors();
    }


    public void moveBackward(double power, int time)
    {
        moveForward(-power, time);

    }

    public void MoveTilesBackFast(double amount)
    {
        moveBackward(0.8, (int)(495 * amount));
        Pause(1);
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);
        RightRear.setPower(0);
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

   public void ViperHighBasketPosition()
   {

       ViperVroomUp(0.8,1900);
   }

   public void ArmHighBasketPosition()
   {
       ArmUp(1.0,1660); //1780

   }

    public void ViperRestFromHighPosition()
    {

        ViperVroomDown(0.8,1300);
    }

    public void OneBasketAttempt() //Green fn
    {
        StrafeTilesRight(1.0); //1.2
        Pause(500);

        TurnLeft(0.5,400); //430 org
        Pause(500);

        ArmHighBasketPosition();
        Pause(500);

        ViperHighBasketPosition();
        Pause(500);

        MoveWrist(WristHighBasketPosition);
        Pause(500);

        MoveTiles(0.55);
        Pause(500);

        MoveTiles(0.1);
        Pause(500);

        clawOpen();
        Pause(500);

        MoveTilesBackwards(0.15);
        Pause(500);

        ViperRestFromHighPosition();
        Pause(500);

        ArmDown(1.0, 950);
        Pause(500);

        TurnRight(0.5,390);
        //TurnRight(0.9,1150);
        Pause(500);

        StrafeTilesRight(1.85);
        //StrafeTilesLeft(1.8);
        Pause(500);

        MoveTilesBackwards(0.5);
        Pause(500);

       // ViperHighBasketPosition();
        //Pause(500);

        //MoveWrist(WristGrabbingPosition);
        //Pause(500);

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

    public void SetArmPosition(double power, int position){

        int targetPosition = (int) (position * countsPerInch);

        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightArmMotor.setTargetPosition(targetPosition);
        LeftArmMotor.setTargetPosition(targetPosition);

        RightArmMotor.setPower(power);
        LeftArmMotor.setPower(power);

        while(RightArmMotor.isBusy() && LeftArmMotor.isBusy()){
            telemetry.addData("Right arm Position", RightArmMotor.getCurrentPosition());
            telemetry.addData("Left arm Position", LeftArmMotor.getCurrentPosition());
            telemetry.update();
        }
        RightArmMotor.setPower(0);
        LeftArmMotor.setPower(0);
    }

    public void SetViperSlidePosition(double power, int position){

        //ViperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int targetPosition = (int) (position * countsPerInch);

        ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ViperMotor.setTargetPosition(targetPosition);

        ViperMotor.setPower(power);

        while(ViperMotor.isBusy())
        {
            telemetry.addData("Viper Slide Position",  ViperMotor.getCurrentPosition());
            telemetry.update();
        }

        ViperMotor.setPower(0);

    }

    public void MoveForwardPosition(double power, int position){

        int targetPosition = (int) (position * countsPerInch);

        RightFront.setTargetPosition(targetPosition);
        LeftFront.setTargetPosition(targetPosition);
        LeftRear.setTargetPosition(targetPosition);
        RightRear.setTargetPosition(targetPosition);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFront.setPower(power);
        LeftFront.setPower(power);
        RightRear.setPower(power);
        LeftRear.setPower(power);

        while(RightFront.isBusy() && LeftFront.isBusy() && RightRear.isBusy() && LeftRear.isBusy())
        {
            telemetry.addData("Right Front Position", RightFront.getCurrentPosition());
            telemetry.addData("Left Front Position", RightFront.getCurrentPosition());
            telemetry.addData("Right Rear Position", RightFront.getCurrentPosition());
            telemetry.addData("Left Rear Position", RightFront.getCurrentPosition());
            telemetry.update();
        }

        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightRear.setPower(0);
        LeftRear.setPower(0);

        TurnLeft(0.3, 500);
    }

    public void ArmDown(double power, int time)
    {


        LeftArmMotor.setPower(power);
        RightArmMotor.setPower(power);
        sleep(time);
        LeftArmMotor.setPower(0);
        RightArmMotor.setPower(0);


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
        LeftGripperServo.setPosition(ClawOpenNormalPosition); // Open Left Gripper
        RightGripperServo.setPosition(ClawOpenNormalPosition); // Open Right Gripper
    }

    public void clawClose()
    {
        LeftGripperServo.setPosition(ClawNomNom); // Close left Gripper
        RightGripperServo.setPosition(ClawNomNom); // Close Right Gripper
    }

    public void MoveWrist(double pos)
    {
        WristServo.setPosition(pos);

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
        Pause(300);
        TurnRight(0.3,63);
        stopMotors();

    }


    public void StrafeTilesRight(double amount)
    {

        StrafeRight(0.3,(int)(2100 * amount));
        Pause(300);
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
