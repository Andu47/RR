package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name= "Brat", group = "LinearOpMode")
public class BRAT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Brat;
        DcMotor FataDr;
        DcMotor FataSt;
        DcMotor SpateDr;
        DcMotor SpateSt;
        DcMotor Intake;
        Servo servo;
        double y = 0, a = 0, b = 0;
        double pozinit=0.8;
        double pozfin=0.3;
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Brat = hardwareMap.get(DcMotor.class, "Brat");
        FataDr = hardwareMap.get(DcMotor.class, "FataDr");
        FataSt = hardwareMap.get(DcMotor.class, "FataSt");
        SpateDr = hardwareMap.get(DcMotor.class, "SpateDr");
        SpateSt = hardwareMap.get(DcMotor.class, "SpateSt");
        servo = hardwareMap.get(Servo.class, "servo");

        FataSt.setDirection(DcMotorSimple.Direction.REVERSE);
        SpateSt.setDirection(DcMotorSimple.Direction.REVERSE);
        Brat.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.x)servo.setPosition(0.3);//sus
            else if(gamepad1.b)servo.setPosition(0.8);//jos
        }
    }
}