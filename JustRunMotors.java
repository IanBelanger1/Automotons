package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Template: JustRunMotors", group="Linear Opmode")
public class JustRunMotors extends LinearOpMode {
    private DcMotor[] motorList;
    @Override
    public void runOpMode() {
        motorList = new DcMotor[4];
        for (int i = 0; i < motorList.length; i ++) {
            motorList[i] = hardwareMap.get(DcMotor.class, "motor" + i);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            for (int i = 0; i < motorList.length; i++) {
                motorList[i].setPower(0.1);
            }
            telemetry.addData("Status", "running");
            telemetry.update();
        }
        telemetry.update();

    }

}

