package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;

@Autonomous(name="no platform blue", group="blue")
public class DriiveAutoBlueNoplatform extends BlinkyAuto {
    public void runOpMode() {
        initRobot(BLUE);

        waitForStart();
        driving.resetZero();

        // Drive to look at the blocks
        driving.polarAuto(autospeed, PI / 2, 460 * clicksPerMm, this, 100);
        driving.stopWheels();

        // Open the side lift grabber
        robot.sideliftgrab.setPosition(0);

        // Give Vuforia a second to locate the block
        int position = vuforiaDetection(2);

        // Set the side lift to RUN_TO_POSITION
        robot.sidelift.setTargetPosition(robot.sidelift.getCurrentPosition());
        robot.sidelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Drive to block
        toBlock(position);

        // Drive across the field
        driving.polarAuto(autospeed, 0, 1200 * clicksPerMm, this, 100, false, false);
        robot.sidelift.setTargetPosition(-1000);
        driving.polarAuto(autospeed, 0, 600 * clicksPerMm, this, 200, false, true);

        // to platform w/ sensor
        toPlatform();

        // let go of block
        robot.sideliftgrab.setPosition(0);
        sleep(200);

        // away from platform
        driving.polarAuto(autospeed, 3 * PI / 2, 200 * clicksPerMm, this);

        // back across field
        robot.sidelift.setTargetPosition(0);
        driving.turnAbs(PI);
        driving.turnSpeed = 1;
        switch(position) {
            case 1:
                driving.polarAuto(autospeed * .6, PI, 2720 * clicksPerMm, this, 100);
            case 2:
                driving.polarAuto(autospeed * .6, PI, 2520 * clicksPerMm, this, 100);
            case 3:
                driving.polarAuto(autospeed * .6, PI, 2320 * clicksPerMm, this, 100);
        }
        driving.turnSpeed = 0.6;

        // to second block
        driving.polarAuto(autospeed * .6, PI / 2, 350 * clicksPerMm, this, 100);
        // grab second block
        robot.leftintake.setPower(-1);
        robot.rightintake.setPower(-1);
        driving.polarAuto(0.1, PI, 1, this, 0, false, false);
        while(robot.intake_button.getState());
        robot.leftintake.setPower(0);
        robot.rightintake.setPower(0);
        //grabBlock(0);

        // towards wall
        driving.polarAuto(autospeed, 3 * PI / 2, 150 * clicksPerMm, this, 100);
        driving.stopWheels();

        driving.turnAbs(0);

        // to other side
        switch(position) {
            case 1:
                driving.polarAuto(autospeed, 0, 2000 * clicksPerMm, this, 100);
            case 2:
                driving.polarAuto(autospeed, 0, 1700 * clicksPerMm, this, 100);
            case 3:
                driving.polarAuto(autospeed, 0, 1500 * clicksPerMm, this, 100);
        }

        robot.tape.setPower(-1);
        robot.leftintake.setPower(1);
        robot.rightintake.setPower(1);
        sleep(1000);
        robot.tape.setPower(0);
        robot.leftintake.setPower(0);
        robot.rightintake.setPower(0);
        //robot.sidelift.setTargetPosition(-1000);
        //driving.polarAuto(autospeed, 0, 600 * clicksPerMm, this, 200);

        //toPlatform();

        //robot.sideliftgrab.setPosition(0);
    }

    public void updateAuto(TelemetryPacket packet) {
        dashboard.sendTelemetryPacket(packet);
        robot.updateGyro();
        driving.gyro(robot.angles.thirdAngle);
    }
}
