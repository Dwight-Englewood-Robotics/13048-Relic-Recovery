package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.*;

public class Bot {
    static DcMotor BL, BR, FL, FR, ARM;
    HardwareMap map;
    Telemetry tele;
    static Servo lClaw, rClaw, colorServo;
    static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;
    Double powerModifier = 0.02;
    ModernRoboticsI2cColorSensor colorSensor;
    public Bot()
    {}

    public void init(HardwareMap map, Telemetry tele, boolean auton){
        this.map = map;
        this.tele = tele;
        BL = this.map.get(DcMotor.class, "BL");
        BR = this.map.get(DcMotor.class, "BR");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        ARM = this.map.get(DcMotor.class, "ARM");
        colorSensor = this.map.get(ModernRoboticsI2cColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);
        if(auton)
            runToPosition();
        else
            runToEncoder();
        ARM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lClaw = map.servo.get("lClaw");
        rClaw = map.servo.get("rClaw");
        colorServo = map.servo.get("colorServo");
//        colorServo = map.servo.get("colorServo");
        ARM.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        ARM.setPower(0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro = map.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        tele.addData(">","Gyro Calibrating. Do Not move!");
        tele.update();
    }
    public void runToPosition(){
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runToEncoder(){
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double y, double x, double r) {
        double bl = - y - x + r;
        double br = y - x + r;
        double fr = -y + x + r;
        double fl = y + x + r;
        BL.setPower(bl);
        BR.setPower(br);
        FR.setPower(fr);
        FL.setPower(fl);
    }

    public void turn(double in){
        BL.setPower(in);
        BR.setPower(in);
        FL.setPower(in);
        FR.setPower(in);
    }
    public void armUp(double in){
            ARM.setPower(in);
    }
    public void armDown(double in){
             ARM.setPower(-in);
    }
     public void openClaw(){
     lClaw.setPosition(0.6);
     rClaw.setPosition(0.4);
    }
    public void closeClaw(){
        lClaw.setPosition(1.0);
        rClaw.setPosition(0.0);
    }
    public void addServo(double add) {
        rClaw.setPosition(rClaw.getPosition() + add);
        lClaw.setPosition(lClaw.getPosition() + add);
    }
    public void decreaseServo(double sub) {
        rClaw.setPosition(rClaw.getPosition() - sub);
        lClaw.setPosition(lClaw.getPosition() - sub);
    }
//    public void resetColor(){
//        colorServo.setPosition(0.0);
//    }
    public void strafe(double in){
        BR.setPower(-in);
        FR.setPower(in);
        BL.setPower(-in);
        FL.setPower(in);
    }
    public void resetEncoders(){
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runToPosition();
    }
    public void setToPosition(int position){
        BL.setTargetPosition(position);
        BR.setTargetPosition(-position);
        FL.setTargetPosition(-position);
        FR.setTargetPosition(position);
    }
  /*  public void fieldCentricDrive(double lStickX, double lStickY, double rStickX) {
        // Get the controller values
        double forward = (-1)*lStickY;
        double right =  lStickX;
        double clockwise = rStickX;
        double temp;
        double k;

        // Apply the turn modifier k
        clockwise *= k;

        // Turn the output heading value to be based on counterclockwise turns
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0) {
            angles.firstAngle += 360;
        }

        // Convert to Radians for Math.sin/cos
        angles.firstAngle = (float)(angles.firstAngle * (Math.PI / 180));

        // Do Math
        temp = forward * Math.cos(angles.firstAngle) - right * Math.sin(angles.firstAngle);
        right = forward * Math.sin(angles.firstAngle) + right * Math.cos(angles.firstAngle);
        forward = temp;

        // Set power values using Math
        frontLeft = forward + clockwise + right;
        frontRight = forward - clockwise - right;
        rearLeft = forward + clockwise - right;
        rearRight = forward - clockwise + right;

        // Clip power values to within acceptable ranges for the motors
        frontLeft = Range.clip(frontLeft, -1.0, 1.0);
        frontRight = Range.clip(frontRight, -1.0, 1.0);
        rearLeft = Range.clip(rearLeft, -1.0, 1.0);
        rearRight = Range.clip(rearRight, -1.0, 1.0);

        // Send power values to motors
        FL.setPower(frontLeft);
        BL.setPower(rearLeft);
        FR.setPower(frontRight);
        BR.setPower(rearRight);
    }
    */

    public boolean adjustHeading(int targetHeading) {
        double curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (Math.abs(Math.abs(targetHeading) - Math.abs(curHeading)) < .5) {
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            return true;
        }
        double headingError;
        if (targetHeading == 0) {
            headingError = curHeading < 0 ? targetHeading + curHeading : Math.abs(targetHeading + curHeading);
        }
        else
            headingError = targetHeading + curHeading;
        double  driveScale = headingError * powerModifier;
        if (Math.abs(driveScale) < .06) {
            driveScale = .06 * (driveScale < 0 ? -1 : 1);
        }
        Range.clip(driveScale, -1, 1);
        turn(driveScale);
        return false;
    }
    public void testServos(Telemetry tele){
        tele.addData("Claw: ", lClaw.getPosition());
        tele.update();
    }
    public void testMotor(Telemetry tele){
        tele.addData("Motor: ", FL.getCurrentPosition());
        tele.update();
    }
    public void upColor(){
        colorServo.setPosition(0.0);
    }
    public void downColor(){
        colorServo.setPosition(1.0);
    }
    public boolean isRed(){
        return colorSensor.red() >= 2;
    }
    public int getMotorPosition(){
        return Math.max(BR.getCurrentPosition(),Math.max(BL.getCurrentPosition(),Math.max(FL.getCurrentPosition(),FR.getCurrentPosition())));
    }
    public int getTargetPosition() {
        return FL.getTargetPosition();}
}
