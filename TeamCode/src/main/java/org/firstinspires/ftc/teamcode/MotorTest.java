package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest", group = "TeleOp")

public class MotorTest extends OpMode {

    private DcMotor HangMotor;

    @Override
    public void init() {
        // Initialize the HangMotor from the hardware map
        HangMotor = hardwareMap.get(DcMotor.class, "HangMotor");

        // Set the motor to brake when power is set to 0
        HangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop() {
        // If 'A' button is pressed, set the motor power to 0.3
        if (gamepad1.left_stick_y > 0.5) {
            HangMotor.setPower(1.0);
        }

        if (gamepad1.left_stick_y < -0.5) {
            HangMotor.setPower(-1.0);
        }

        HangMotor.setPower(0);

        telemetry.addData("HangMotor Power", HangMotor.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the motor when the OpMode is stopped
        HangMotor.setPower(0);
    }
}
