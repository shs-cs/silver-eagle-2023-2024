package org.firstinspires.ftc.teamcode;
// Make sure the wrist is tightened before use or it tends to be janky
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "WormMotor", group = "TeleOp")
public class WormMotor extends OpMode {

    private DcMotor WormMotor;

    @Override
    public void init() {
        WormMotor = hardwareMap.dcMotor.get("WormMotor");
        WormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Call the method that controls the motor in each loop cycle
        WormMotorVroom();
    }

    // Method to control the WormMotor based on joystick input
    public void WormMotorVroom() {
        // Check if joystick is moved up or down
        if (gamepad2.left_stick_y > 0.5) {
            WormMotor.setPower(1.0);  // Move motor forward
        } else if (gamepad2.left_stick_y < -0.5) {
            WormMotor.setPower(-1.0); // Move motor backward
        } else {
            WormMotor.setPower(0);    // Stop the motor if joystick is neutral
        }

        // Display the motor's power in telemetry
        telemetry.addData("WormMotor Power", WormMotor.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        int targetPosition = 0;

        // Set the motor's target position
        WormMotor.setTargetPosition(targetPosition);

        // Set the mode to run to position
        WormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power to initiate the movement
        WormMotor.setPower(0.5);  // You can adjust this power as needed

        // Stop the motor
        WormMotor.setPower(0);
    }
}

