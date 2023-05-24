package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class mecanisme{

    DcMotor slide;
    Servo grip, pivot, swing;


    public mecanisme(HardwareMap hardwareMap){
        slide = hardwareMap.get(DcMotor.class, "slide");
        grip = hardwareMap.get(Servo.class, "grip");
        pivot = hardwareMap.get(Servo.class, "pivot");
        swing = hardwareMap.get(Servo.class,"swing");

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void slidePosition (int position, double power) {
        slide.setTargetPosition(position);
        slide.setPower(power);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}
