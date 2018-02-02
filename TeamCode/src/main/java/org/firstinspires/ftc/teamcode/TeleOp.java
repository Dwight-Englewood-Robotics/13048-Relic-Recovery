package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Bot;
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

import org.firstinspires.ftc.teamcode.Bot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleop",group="Teleop")
//@Disabled
public class TeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    Bot robot = new Bot();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
//        robot.resetServo();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {timer.reset();}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        else if((gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15)) {
//            robot.strafe((double) gamepad1.right_stick_x);
//            telemetry.addData("turn: ", true);
//            telemetry.update();
//        }
//        else if((gamepad1.right_trigger > 0.15))
//            robot.turn(1.0);
//        else if(gamepad1.left_trigger > 0.15)
//            robot.turn(-1.0);
        if((gamepad1.left_stick_y < 0.15 && gamepad1.left_stick_y > -0.15) && (gamepad1.left_stick_x < 0.15 && gamepad1.left_stick_x > -0.15) && (gamepad1.right_stick_x < 0.15 && gamepad1.right_stick_x > -0.15)){
            robot.drive(0.0, 0.0, 0.0);
            robot.turn(0.0);
        }
        else {
            robot.drive((double) -gamepad1.left_stick_y, (double) gamepad1.left_stick_x, (double) gamepad1.right_stick_x);
            telemetry.addData("drive: ", true);
            telemetry.update();
        }
        if(gamepad2.right_trigger > 0.15)
            robot.armUp((double) (gamepad2.right_trigger));
        else if(gamepad2.left_trigger > 0.15)
            robot.armDown((double) (gamepad2.left_trigger));
        else {
            robot.armDown(0.0);
            robot.armUp(0.0);
//            telemetry.addData("PositionsFR: ", Bot.FR.getCurrentPosition());
//            telemetry.addData("PositionsFL: ", Bot.FL.getCurrentPosition());
//            telemetry.addData("PositionsBR: ", Bot.BR.getCurrentPosition());
//            telemetry.addData("PositionsBL: ", Bot.BL.getCurrentPosition());
        }
//        telemetry.addData("Motor Position: ", robot.ARM.getCurrentPosition());
        if(gamepad2.a) {
            telemetry.addData("Is A down: ", 0.00);
            robot.closeClaw();
        }
        else if(gamepad2.b) {
            telemetry.addData("Is B down: ", 1.00);
            robot.openClaw();
        }
        if(gamepad2.x){
            robot.upColor();
            telemetry.addData("UpColor: ", true);
        }
        else if(gamepad2.y){
            robot.downColor();
            telemetry.addData("DownColor: ", true);
        }
        if(gamepad1.x){
            while(true) {
                robot.adjustHeading(90);
                if (90 - Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 5) {
                    break;
                }
                telemetry.addData("Degrees90: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }
        }
        if(gamepad1.y){
            while(true) {
                robot.adjustHeading(-90);
                if (90 - Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 5) {
                    break;
                }
                telemetry.addData("Degrees-90: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }
        }
        telemetry.addData("rClaw: ", robot.rClaw.getPosition());
        telemetry.addData("lClaw: ", robot.lClaw.getPosition());
        telemetry.update();
//        telemetry.addData("degrees: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        telemetry.update();
//        robot.testServos(telemetry);
//        telemetry.update();
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
