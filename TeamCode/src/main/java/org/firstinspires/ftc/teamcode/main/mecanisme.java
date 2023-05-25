package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class mecanisme{

    DcMotor slide;
    Servo grip, pivot, turn, sula;

    double Gripper_OPEN = 1;
    double Gripper_CLOSE = 0.5;
    double Pivot_UP = 0.2;
    double Pivot_DOWN = 0.3;

    double Pivot_SusDeTot = 0.8;
    double Turn_FRONT = 0.5;
    double Turn_RIGHT = 0.4;
    double Turn_LEFT = 0.6;

    int pos = 0;
    int LOW  = -410;
    int MID = -820;
    int HIGH = -1155;

    public mecanisme(HardwareMap hardwareMap){
        slide = hardwareMap.get(DcMotor.class, "slide");
        grip = hardwareMap.get(Servo.class, "grip");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class,"swing");
        sula = hardwareMap.get(Servo.class,"sula");

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void slidePosition (int position, double power) {
        slide.setTargetPosition(position);
        slide.setPower(power);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void gripper() {
        if (grip.getPosition() >= Gripper_OPEN) {
            grip.setPosition(Gripper_CLOSE);
        } else {
            grip.setPosition(Gripper_OPEN);
        }
    }

}
