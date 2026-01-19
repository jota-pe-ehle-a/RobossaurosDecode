package org.firstinspires.ftc.teamcode.DECODE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


@Autonomous(name = "AutoAzulMaior")
public class AutoAzulMaior extends LinearOpMode {
    Follower follower;
    DcMotor motorColetor, motorColetor2, lancador;
    Telemetry telemetryA;
    double voltagem;


    int etapaAtual = 0;
    int obelisko = 0;

    Pose startPose        =  new Pose(18,123,Math.toRadians(143));
    Pose poseDeLancamento =  new Pose(48,96,Math.toRadians(140));
    Pose poseDeVisao      =  new Pose(51,96,Math.toRadians(60));
    Pose poseFinal        =  new Pose(72,48,Math.toRadians(90));

    Pose coleta21         =  new Pose(42,36,0);
    Pose coleta21Final    =  new Pose(22,36,0);
    Pose coleta21Ctrl     =  new Pose(48,36);

    Pose coleta22         =  new Pose(42,60,0);
    Pose coleta22Final    =  new Pose(22,60,0);
    Pose coleta22Ctrl     =  new Pose(48,60);

    Pose coleta23         =  new Pose(50,81,0);
    Pose coleta23Final    =  new Pose(35,81,0);
    Pose coleta23Ctrl     =  new Pose(48,81);

    Path pathStart   =  criarLinha(startPose,poseDeLancamento);
    Path pathOlhar   =  criarLinha(poseDeLancamento,poseDeVisao);
    Path pathFinal   =  criarLinha(poseDeLancamento,poseFinal);

    Path path21_1    =  criarCurva(poseDeLancamento,coleta21Ctrl,coleta21);
    Path path21_2    =  criarLinha(coleta21,coleta21Final);
    Path path21_3    =  criarCurva(coleta21Final,coleta21Ctrl,poseDeLancamento);

    Path path22_1    =  criarCurva(poseDeLancamento,coleta22Ctrl,coleta22);
    Path path22_2    =  criarLinha(coleta22,coleta22Final);
    Path path22_3    =  criarCurva(coleta22Final,coleta22Ctrl,poseDeLancamento);

    Path path23_1    =  criarLinha(poseDeLancamento,coleta23);
    Path path23_2    =  criarLinha(coleta23,coleta23Final);
    Path path23_3    =  criarCurva(coleta23Final,coleta23Ctrl,poseDeLancamento);

    @Override
    public void runOpMode() throws InterruptedException {
        boolean truE = true;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        motorColetor  = hardwareMap.dcMotor.get("motorCO");
        motorColetor2 = hardwareMap.dcMotor.get("motorCO2");
        lancador      = hardwareMap.dcMotor.get("motorLancador");
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        pathStart.setLinearHeadingInterpolation(startPose.getHeading(),poseDeLancamento.getHeading());
        pathOlhar.setLinearHeadingInterpolation(poseDeLancamento.getHeading(),poseDeVisao.getHeading());
        pathFinal.setLinearHeadingInterpolation(poseDeLancamento.getHeading(),poseFinal.getHeading());

        path21_1.setLinearHeadingInterpolation(poseDeVisao.getHeading(),0);
        path21_2.setConstantHeadingInterpolation(0);
        path21_3.setLinearHeadingInterpolation(0, poseDeLancamento.getHeading());

        path22_1.setLinearHeadingInterpolation(poseDeLancamento.getHeading(),0);
        path22_2.setConstantHeadingInterpolation(0);
        path22_3.setLinearHeadingInterpolation(0, poseDeLancamento.getHeading());

        path23_1.setLinearHeadingInterpolation(poseDeLancamento.getHeading(),0);
        path23_2.setConstantHeadingInterpolation(0);
        path23_3.setLinearHeadingInterpolation(0, poseDeLancamento.getHeading());

        CameraDecode cameraDecode = new CameraDecode(hardwareMap);


        waitForStart();
        while (opModeIsActive() && truE){
            salvarPosicaoDoRobo(follower);
            follower.update();
            telemetryA.addData("Pose do robo(X,Y): ", follower.getPose());
            telemetryA.update();
            switch (etapaAtual){
                case 0:
                    follower.followPath(pathStart);
                    if(follower.atParametricEnd()){
                        voltagem = hardwareMap.voltageSensor.iterator().next().getVoltage();
                        lancarArtefato(calcularPotencia());
                        etapaAtual++;
                        obelisko = 23;
                    }
                    break;
                case 1:
                    if(obelisko == 21){
                        follower.followPath(path21_1);
                    } else if(obelisko == 22){
                        follower.followPath(path22_1);
                    } else if(obelisko == 23){
                        follower.followPath(path23_1);
                    }
                    if(follower.atParametricEnd()){
                        etapaAtual++;
                        sleep(500);
                        coletar(false);
                    }
                    break;
                case 2:
                    if(obelisko == 21){
                        follower.followPath(path21_2);
                    } else if(obelisko == 22){
                        follower.followPath(path22_2);
                    } else if(obelisko == 23){
                        follower.followPath(path23_2);
                    }
                    if(follower.atParametricEnd()){
                        coletar(true);
                        etapaAtual++;
                    }
                    break;
                case 3:
                    if(obelisko == 21){
                        follower.followPath(path21_3);
                        if(follower.atParametricEnd()){
                            lancarArtefato(calcularPotencia());
                            etapaAtual++;
                        }
                    } else if(obelisko == 22){
                        follower.followPath(path22_3);
                        if(follower.atParametricEnd()){
                            lancarArtefato(calcularPotencia());
                            etapaAtual++;
                        }
                    } else if(obelisko == 23){
                        follower.followPath(path23_3);
                        if(follower.atParametricEnd()){
                            lancarArtefato(calcularPotencia());
                            etapaAtual++;
                        }
                    }
                    break;
                case 4:
                    follower.followPath(pathFinal);
                    if(follower.atParametricEnd()){
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
        sleep(2800);
        motorColetor2.setPower(1);
        motorColetor.setPower(1);
        sleep(400);
        motorColetor.setPower(0);
        motorColetor2.setPower(0);
        sleep(1500);
        motorColetor2.setPower(1);
        motorColetor.setPower(1);
        sleep(1000);
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
            sleep(200);
            motorColetor2.setPower(0);
        }
    }
    Path criarCurva(Pose startPose,Pose ctrlPose, Pose finalPose){
        return new Path(
                new BezierCurve(
                        startPose,ctrlPose,finalPose
                )
        );
    }
    Path criarLinha(Pose startPose,Pose finalPose){
        return new Path(
                new BezierLine(
                        startPose,finalPose
                )
        );
    }
    void salvarPosicaoDoRobo(Follower follower){
        double xAgora = follower.getPose().getX();
        double yAgora = follower.getPose().getY();
        double hAgora = follower.getPose().getHeading();

        String fileName = "current_robot_position.cvc";

        File internalDir = hardwareMap.appContext.getFilesDir();

        File file = new File(internalDir,fileName);
        String dataToSave = xAgora + "," + yAgora + "," + hAgora;

        try{
            FileWriter writer = new FileWriter(file);
            writer.write(dataToSave);
            writer.close();

            telemetryA.addData("Pose a ser salva: ", "X: %.2f , Y: %.2f , Heading: %.2f",xAgora,yAgora,hAgora);
        } catch (IOException e){
            telemetryA.addData("Não foi possível salvar a pose do robô","ERROR:" + e.getMessage());
        }
    }
    double calcularPotencia(){
        return 0.6*13.5/voltagem;
    }
}
