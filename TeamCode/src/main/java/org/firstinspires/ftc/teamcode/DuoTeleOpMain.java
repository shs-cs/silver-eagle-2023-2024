package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DuoTeleOpMain", group = "TeleOp")
public class DuoTeleOpMain extends OpMode {
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftRear;
    private DcMotor RightRear;
    private DcMotor TurretMotor;
    private DcMotor WormMotor;
    private DcMotor IntakeMotor;
    private Servo AirplaneServo;
    private Servo WristServo;
    private Servo GripperServo;


    private float lastWristPosition = 0;
    private float lastClawPosition = 0;


    public static final double GRIPPER_CLOSED_POSITION = 1.0;
    public static final double GRIPPER_OPENED_POSITION = 0.90;



    @Override
    public void init() {



        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        LeftRear = hardwareMap.dcMotor.get("LeftRear");
        RightRear = hardwareMap.dcMotor.get("RightRear");
        TurretMotor = hardwareMap.dcMotor.get("TurretMotor");
        WormMotor = hardwareMap.dcMotor.get("WormMotor");
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        AirplaneServo = hardwareMap.servo.get("AirplaneServo");
        GripperServo = hardwareMap.servo.get("GripperServo");
        WristServo = hardwareMap.servo.get("WristServo");

        //GripperServo.setDirection(Servo.Direction.REVERSE);
        GripperServo.setPosition(GRIPPER_CLOSED_POSITION);



        // Reverse the direction of the right motors if needed
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);

        // Set the motors to brake when power is set to 0
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        handleDriveTrain();
        turretMotor();
        handleIntakeMotor();
        handleWormMotor();
        handleAirplaneServo();
        handleWristServo();
        handleGripperServo();

        telemetry.addLine("AirplaneServo Position:" + AirplaneServo.getPosition());
        telemetry.addLine("Right trigger:" + gamepad2.right_trigger);
        telemetry.addLine("Left trigger:" + gamepad2.left_trigger);
        telemetry.addLine("lastWristPosition :" + lastWristPosition);
        telemetry.addLine("--------------------");
        telemetry.addLine("Right bumper:" + gamepad2.right_bumper);
        telemetry.addLine("Left bumper:" + gamepad2.left_bumper);
        telemetry.addLine("--------------------");
        telemetry.addLine("Gripper Servo Position:" + GripperServo.getPosition());

    }

    public void handleAirplaneServo() {

        if (gamepad1.y) {
            AirplaneServo.setPosition(0.335); // Set position to halfway (0.0 to 1.0)
        }


        if (gamepad1.x) {
            AirplaneServo.setPosition(0.0); // Set position to halfway (0.0 to 1.0)
        }


    }



    public void handleWristServo() //throws InterruptedException
    {
        double currentPosition = WristServo.getPosition();
        if (gamepad2.y) {
            WristServo.setPosition(0.345); // Set position to halfway (0.0 to 1.0)
            //LinearOpMode.sleep(500);
        }



        if (gamepad2.b) {

            double newPosition = currentPosition + 0.00005;
            // Make sure the newPosition is within the valid range for the servo
            newPosition = Math.min(1.0, Math.max(0.0, newPosition)); // Limit within [0, 1]
            WristServo.setPosition(newPosition);
        }




        if (gamepad2.x) {
            WristServo.setPosition(0.0); // Set position to halfway (0.0 to 1.0)
            //LinearOpMode.sleep(500);
        }



        if (gamepad2.a) {

            double newPosition = currentPosition - 0.00005;
            // Make sure the newPosition is within the valid range for the servo
            newPosition = Math.min(1.0, Math.max(0.0, newPosition)); // Limit within [0, 1]
            WristServo.setPosition(newPosition);
        }

    }



    public void handleGripperServo()
    {

        if (gamepad2.right_bumper) {

            GripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        }


        if (gamepad2.right_trigger > 0.5) { // Assuming a threshold of 0.5 for the right trigger
            double currentPosition = GripperServo.getPosition();
            double newPosition = currentPosition + 0.001;
            // Make sure the newPosition is within the valid range for the servo
            newPosition = Math.min(1.0, Math.max(0.0, newPosition)); // Limit within [0, 1]
            GripperServo.setPosition(newPosition);
        }



        if (gamepad2.left_bumper) {

            GripperServo.setPosition(GRIPPER_OPENED_POSITION);
        }


        if (gamepad2.left_trigger > 0.5) { // Assuming a threshold of 0.5 for the right trigger
            double currentPosition = GripperServo.getPosition();
            double newPosition = currentPosition - 0.001;
            // Make sure the newPosition is within the valid range for the servo
            newPosition = Math.min(1.0, Math.max(0.0, newPosition)); // Limit within [0, 1]
            GripperServo.setPosition(newPosition);
        }


    }






    public void handleDriveTrain() {
        double leftStickY = gamepad1.left_stick_y; // Invert Y axis
        double leftStickX = -gamepad1.left_stick_x;
        double rightStickX = -gamepad1.right_stick_x;

        double powerLF = getPower(leftStickY + leftStickX + rightStickX);
        double powerRF = getPower(leftStickY - leftStickX - rightStickX);
        double powerLR = getPower(leftStickY - leftStickX + rightStickX);
        double powerRR = getPower(leftStickY + leftStickX - rightStickX);

        LeftFront.setPower(powerLF);
        RightFront.setPower(powerRF);
        LeftRear.setPower(powerLR);
        RightRear.setPower(powerRR);
    }


    public void turretMotor() {
        // Turret control
        double turretPower = 0.3 * getPower(gamepad2.right_stick_x);

        // Check if the right stick X value has changed
        if (turretPower != 0) {
            // Move the turret as usual
            TurretMotor.setPower(getPower(turretPower));
        } else {
            // If the left stick is released, stop the turret instantly
            TurretMotor.setPower(0);
        }
    }



    public void handleWormMotor() {
        // Worm control with a maximum speed of 50%
        double WormPower = -0.85 * getPower(gamepad2.left_stick_y);

        // Move the worm motor with the adjusted power
        if (WormPower != 0) {
            // Move the turret as usual
            WormMotor.setPower(WormPower);
        } else {
            // If the left stick is released, stop the turret instantly
            WormMotor.setPower(0);
        }
    }




    public void handleIntakeMotor() {
        // Intake control with gamepad1 bumpers
        if (gamepad1.right_bumper) {
            // Spin the intake motor forward
            IntakeMotor.setPower(1.0); // Adjust power as needed
        } else if (gamepad1.left_bumper) {
            // Stop the intake motor
            IntakeMotor.setPower(0);
        }
        else if (gamepad1.b)
        {
            IntakeMotor.setPower(-1.0);
        }

        // Add any additional logic or controls as needed
    }


    /**
     * This method takes a power level and throttles
     * it if it's too high. This is so we have more
     * control over our components.
     *
     * All setPower motor calls should call this
     */
    public double getPower(double powerLevel) {
        // If absolute value of power level is less than 0.75
        // return passed in power level
        if ((powerLevel < 0.75 && powerLevel >= 0 )|| (powerLevel <= 0 && powerLevel >= -0.75)) {
            return powerLevel;
        }

        // If power level is negative
        if (powerLevel < 0) {
            return -0.75;
        }

        return 0.75;
    }

    @Override
    public void stop() {
        // Stop the motors when the OpMode is stopped
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);
        RightRear.setPower(0);
        TurretMotor.setPower(0);
        WormMotor.setPower(0);
        IntakeMotor.setPower(0);
        AirplaneServo.setPosition(0);
    }
}