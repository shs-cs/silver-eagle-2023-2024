package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "ArmTest", group = "TeleOp")
public class ArmTest extends OpMode  {


    private DcMotor LeftArmMotor;
    private DcMotor RightArmMotor;

    @Override
    public void init() {
        LeftArmMotor = hardwareMap.dcMotor.get("LeftArmMotor");
        RightArmMotor = hardwareMap.dcMotor.get("RightArmMotor");
        //LeftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop()
    {
        ArmGoVroom();
    }

    public void ArmGoVroom()
    {



        if (gamepad2.left_stick_y < -0.5)
        {
            LeftArmMotor.setPower(-1.0);
            RightArmMotor.setPower(-1.0);
        }

        LeftArmMotor.setPower(0);
        RightArmMotor.setPower(0);


        if (gamepad2.left_stick_y > 0.5)
        {
            LeftArmMotor.setPower(1.0);
            RightArmMotor.setPower(1.0);
        }









    }

    @Override
    public void stop() {

        LeftArmMotor.setPower(0);
        RightArmMotor.setPower(0);
    }
}


