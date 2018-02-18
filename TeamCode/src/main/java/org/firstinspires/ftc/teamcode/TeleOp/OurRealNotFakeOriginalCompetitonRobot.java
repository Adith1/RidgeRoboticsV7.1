package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Central;

/**
 * Created by adith on 2/17/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Central;


    /**
     * Created by adith on 2/17/2018.
     */

    @TeleOp(name="FakeBot Test", group = "Main")


    public class OurRealNotFakeOriginalCompetitonRobot extends Central {
        DcMotor rightMotor;
        DcMotor leftMotor;

        public static final String rightMotorS = "rightMotor";
        public static final String leftMotorS= "leftMotor";
        public DcMotor motor(DcMotor motor, HardwareMap hardwareMap, String name, DcMotor.Direction direction) throws InterruptedException{
            motor = hardwareMap.dcMotor.get(name);
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setPower(0);
            return motor;
        }



        @Override
        public void runOpMode() throws InterruptedException {
            waitForStart();
            while(opModeIsActive()) {
                float rightPower = gamepad1.right_stick_y;
                float leftPower = gamepad1.left_stick_y;
                motor(rightMotor, hardwareMap, rightMotorS, DcMotorSimple.Direction.FORWARD).setPower(rightPower);
                motor(leftMotor, hardwareMap, leftMotorS, DcMotorSimple.Direction.FORWARD).setPower(leftPower);
            }
        }











    }


