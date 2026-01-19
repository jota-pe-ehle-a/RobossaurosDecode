package org.firstinspires.ftc.teamcode.DECODE;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constantes.FConstants;
import org.firstinspires.ftc.teamcode.Constantes.LConstants;


@Autonomous(name = "AutoVermelho")
public class AutoVermelhoMenor extends LinearOpMode {
    Follower follower;
    DcMotor motorColetor, motorColetor2, lancador;
    Telemetry telemetryA;

    int etapaAtual = 0;
    int obelisko = 0;

    Pose    startPose           =   new Pose(81, 8,Math.toRadians(90)),
            poseDeLancamento1   =   new Pose(77,17,Math.toRadians(70)),
            poseDeVisao         =   new Pose(80,20,Math.toRadians(95)),
            poseDeLancamento2   =   new Pose(96,96,Math.toRadians(230)),
            poseFinal           =   new Pose(72,48,Math.toRadians(90));
    Path pathStart= new Path(
            new BezierLine(
                    startPose,poseDeLancamento1
            )
    );
    Path pathOlhar= new Path(
            new BezierLine(
                    poseDeLancamento1,poseDeVisao
            )
    );
    Path path23_1 = new Path(
            new BezierCurve(
                    poseDeVisao,new Pose(48,95,Math.toRadians(180))
            )
    );
    Path path23_2 = new Path(
            new BezierLine(
                    new Pose(48,95,Math.toRadians(0)),new Pose(134,84,Math.toRadians(0))
            )
    );
    Path path23_3 = new Path(
            new BezierCurve(
                    new Pose(134,84,Math.toRadians(0)),new Pose(48,96),poseDeLancamento2
            )
    );
    Path path22_1 = new Path(
            new BezierCurve(
                    poseDeVisao,new Pose(69,44),new Pose(81,59,Math.toRadians(180))
            )
    );
    Path path22_2 = new Path(
            new BezierLine(
                    new Pose(81,59,Math.toRadians(0)),new Pose(130,59,Math.toRadians(180))
            )
    );
    Path path22_3 = new Path(
            new BezierCurve(
                    new Pose(130,59,Math.toRadians(0)),new Pose(72,120),poseDeLancamento2
            )
    );
    Path path21_1 = new Path(
            new BezierLine(
                    poseDeVisao,new Pose(83,33,Math.toRadians(180))
            )
    );
    Path path21_2 = new Path(
            new BezierLine(
                    new Pose(83,33,Math.toRadians(0)),new Pose(130,33,Math.toRadians(180))
            )
    );
    Path path21_3 = new Path(
            new BezierCurve(
                    new Pose(130,33,Math.toRadians(0)),new Pose(83,33),poseDeLancamento1
            )
    );
    @Override
    public void runOpMode() throws InterruptedException {
        boolean truE = true;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorColetor  = hardwareMap.dcMotor.get("motorCO");
        motorColetor2 = hardwareMap.dcMotor.get("motorCO2");
        lancador      = hardwareMap.dcMotor.get("motorLancador");
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        CameraDecode cameraDecode = new CameraDecode(hardwareMap);

        pathStart.setLinearHeadingInterpolation(startPose.getHeading(),poseDeLancamento1.getHeading());
        pathOlhar.setLinearHeadingInterpolation(poseDeLancamento1.getHeading(),poseDeVisao.getHeading());

        path21_1.setLinearHeadingInterpolation(poseDeVisao.getHeading(),Math.toRadians(180));
        path21_2.setConstantHeadingInterpolation(Math.toRadians(180));
        path21_3.setLinearHeadingInterpolation(0,poseDeLancamento1.getHeading());

        path22_1.setLinearHeadingInterpolation(poseDeVisao.getHeading(),Math.toRadians(180));
        path22_2.setConstantHeadingInterpolation(Math.toRadians(0));
        path22_3.setLinearHeadingInterpolation(0,poseDeLancamento2.getHeading());

        path23_1.setLinearHeadingInterpolation(poseDeVisao.getHeading(),Math.toRadians(180));
        path23_2.setConstantHeadingInterpolation(Math.toRadians(0));
        path23_3.setLinearHeadingInterpolation(0,poseDeLancamento2.getHeading());




        waitForStart();
        while (opModeIsActive() && truE){
            follower.update();
            switch (etapaAtual){
                case 0:
                    follower.followPath(pathStart);
                    if(follower.atParametricEnd()){
                        lancarArtefato(1);
                        etapaAtual=2;
                        obelisko=21;
                    }
                    break;
                case 1:
                    follower.followPath(pathOlhar);
                    if(cameraDecode.getDetection() != null){
                        if(cameraDecode.getDetection().id != 24 && cameraDecode.getDetection().id != 20){
                            obelisko = cameraDecode.getDetection().id;
                            etapaAtual++;
                        }
                    }
                    break;
                case 2:
                    cameraDecode.stop();
                    if(obelisko == 21){
                        follower.followPath(path21_1);
                    } else if(obelisko == 22){
                        follower.followPath(path22_1);
                    } else if(obelisko == 23){
                        follower.followPath(path23_1);
                    }
                    if(follower.atParametricEnd()){
                        coletar(false);
                        sleep(500);
                        etapaAtual++;
                    }
                    break;
                case 3:
                    if(obelisko == 21){
                        follower.followPath(path21_2);
                    } else if (obelisko == 22){
                        follower.followPath(path22_2);
                    } else if(obelisko == 23){
                        follower.followPath(path23_2);
                    }
                    if(follower.atParametricEnd()){
                        coletar(true);
                        etapaAtual++;
                    }
                    break;
                case 4:
                    if(obelisko == 21){
                        follower.followPath(path21_3);
                        if (follower.atParametricEnd()){
                            lancarArtefato(0.8);
                        }
                    } else if(obelisko == 22){
                        follower.followPath(path22_3);
                        if(follower.atPose(poseDeLancamento2,0.02,0.02)){
                            lancarArtefato(0.5);
                        }
                    } else if(obelisko == 23){
                        follower.followPath(path23_3);
                        if(follower.atPose(poseDeLancamento1,0.02,0.02)){
                            lancarArtefato(0.65);
                        }
                    }
                    if(follower.atParametricEnd()){
                        etapaAtual++;
                    }
                    break;
                case 5:
                    cameraDecode.close();
                    follower.followPath(new Path(
                            new BezierLine(
                                    poseDeLancamento1,poseFinal
                            )
                    ));
                    if(follower.atPose(poseFinal,0.05,0.05)){
                        etapaAtual = -1;
                        truE = false;
                    }
                    break;
                default:
                    break;

            }
        }
    }
    void lancarArtefato(double potencia){
        lancador.setPower(potencia);
        sleep(3000);
        motorColetor2.setPower(1);
        motorColetor.setPower(1);
        sleep(3000);
        lancador.setPower(0);
        motorColetor2.setPower(0);
        motorColetor.setPower(0);
    }
    void coletar(boolean parar) {
        if(!parar){
            motorColetor.setPower(1);
            motorColetor2.setPower(1);
        } else {
            motorColetor.setPower(0);
            motorColetor2.setPower(-0.2);
            sleep(100);
            motorColetor2.setPower(0);
        }
    }
}
