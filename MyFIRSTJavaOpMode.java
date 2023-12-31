package org.firstinspires.ftc.Automotons;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Template: 2024 LiftOnly 1", group="Linear Opmode")
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private DcMotor lfd;
    private DcMotor lrd;
    private DcMotor rfd;
    private DcMotor rrd;
    private DcMotor lfl;
    private DcMotor lrl;
    private DcMotor rfl;
    private DcMotor rrl;
    private int lMod = -1; // changes CW to CCW for left drive motors
    private int rMod = 1; // changes CW to CCW for right drive motors
    @Override
    public void runOpMode() {
        // currently drive motors are disabled b/c we haven't been testing them yet
        /*lfd = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        lrd = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rfd = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rrd = hardwareMap.get(DcMotor.class, "rightRearDrive");*/

        lfl = hardwareMap.get(DcMotor.class, "leftFrontLift");
        lrl = hardwareMap.get(DcMotor.class, "leftRearLift");
        rfl = hardwareMap.get(DcMotor.class, "rightFrontLift");
        rrl = hardwareMap.get(DcMotor.class, "rightRearLift");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y * -1;
            double rightY = gamepad1.right_stick_y * -1;
            telemetry.addData("leftX", "" + leftX);
            telemetry.addData("leftY", "" + leftY);
            telemetry.addData("rightY", "" + rightY);
            if (leftX == 0.0 && leftY == 0.0) {
                double rDepth = gamepad1.right_trigger;
                double lDepth = gamepad1.left_trigger;
                //rotate(lDepth, rDepth);
            } else {
                //translate(leftX, leftY);
            }
            lift(rightY);
            telemetry.update();
        }

    }
    public double pythag(double num1, double num2) {
        return Math.sqrt(Math.pow(num1, 2) + Math.pow(num2, 2));
    }
    public void translate(double xVal, double yVal) {
        double totalPower = pythag(xVal, yVal);
        if (totalPower > 1) totalPower = 1;
        if (totalPower < -1) totalPower = -1;
        if (xVal == 0.0 && yVal == 0.0) {
            lfd.setPower(0.0);
            lrd.setPower(0.0);
            rfd.setPower(0.0);
            rrd.setPower(0.0);
        } else if (xVal >= 0.0 && yVal >= 0.0) {
            lrd.setPower(lMod * totalPower);
            rfd.setPower(rMod * totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(lMod * oppPower);
            rrd.setPower(rMod * oppPower);
        } else if (xVal <= 0.0 && yVal >= 0.0) {
            lfd.setPower(lMod * totalPower);
            rrd.setPower(rMod * totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(lMod * oppPower);
            rfd.setPower(rMod * oppPower);
        } else if (xVal <= 0.0 && yVal <= 0.0) {
            lrd.setPower(-1 * lMod * totalPower);
            rfd.setPower(-1 * rMod * totalPower);
            double oppPower = yVal - xVal;
            lfd.setPower(lMod * oppPower);
            rrd.setPower(rMod * oppPower);
        } else {
            lfd.setPower(-1 * lMod * totalPower);
            rrd.setPower(-1 * rMod * totalPower);
            double oppPower = yVal + xVal;
            lrd.setPower(lMod * oppPower);
            rfd.setPower(rMod * oppPower);
        }
    }
    public void lift(double power) {
        lfl.setPower(-1 * power);
        lrl.setPower(-1 * power);
        rfl.setPower(power);
        rrl.setPower(power);
    }
    public void rotate(double lDepth, double rDepth) {
        if (rDepth > 0) {
            lfd.setPower(lMod * rDepth);
            lrd.setPower(lMod * rDepth);
            rfd.setPower(-1 * rMod * rDepth);
            rrd.setPower(-1 * rMod * rDepth);
        } else if (lDepth > 0){
            lfd.setPower(-1 * lMod * lDepth);
            lrd.setPower(-1 * lMod * lDepth);
            rfd.setPower(rMod * lDepth);
            rrd.setPower(rMod * lDepth);
        } else {
            lfd.setPower(0.0);
            lrd.setPower(0.0);
            rfd.setPower(0.0);
            rrd.setPower(0.0);
        }
    }

}

