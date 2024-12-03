package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends OpMode
{

    private Servo LeftGripperServo;
    private Servo RightGripperServo;
    private Servo WristServo;
    private Servo TwistyTurnyServo;


    @Override
    public void init() {
        LeftGripperServo = hardwareMap.get(Servo.class, "LeftGripperServo");
        TwistyTurnyServo = hardwareMap.get(Servo.class, "TwistyTurnyServo");
        //RightGripperServo.setDirection(Servo.Direction.REVERSE); // Reverse the left gripper
        WristServo = hardwareMap.get(Servo.class, "WristServo");
        RightGripperServo = hardwareMap.get(Servo.class, "RightGripperServo");
        RightGripperServo.setDirection(Servo.Direction.REVERSE);// Default forward, so no need to set it
        // WristServo.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        ServoTest();
    }

    public void ServoTest() {



        if (gamepad1.a) {
            TwistyTurnyServo.setPosition(0.0);
        }

        if (gamepad1.b) {
            WristServo.setPosition(0.0);
        }
















        // Update telemetry to see the position values
        telemetry.update();
    }
}
