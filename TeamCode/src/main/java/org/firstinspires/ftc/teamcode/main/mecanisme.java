package org.firstinspires.ftc.teamcode.main;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class mecanisme{

    boolean ok = false;
    public DcMotor slide;
    public Servo grip, pivot, turn, hopa;

    public double Gripper_OPEN = 0.4;
    public double Gripper_CLOSE = 0.7;
    public double Pivot_SusDeTot = 0.30;
    public double Pivot_UP = 0.65;
    public double Pivot_DOWN = 0.70;
    public double Turn_FRONT = 0.5;
    public double Turn_RIGHT = 0.397;
    public double Turn_LEFT = 0.59;

    public mecanisme(HardwareMap hardwareMap){
        slide = hardwareMap.get(DcMotor.class, "slide");
        grip = hardwareMap.get(Servo.class, "grip");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class,"turn");
        hopa = hardwareMap.get(Servo.class,"sula");

        grip.setPosition(Gripper_CLOSE);
        pivot.setPosition(Pivot_DOWN);
        turn.setPosition(Turn_FRONT);
        hopa.setPosition(0.7);

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

    public void gripper(boolean x) throws InterruptedException{
        if(x){
            if(ok){
                grip.setPosition(Gripper_CLOSE);
                ok = false;
                sleep(150);
            }
            else {

                grip.setPosition(Gripper_OPEN);
                ok = true;
                sleep(150);
            }
        }
    }
}
