package org.firstinspires.ftc.teamcode.DECODE;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.Constantes.FConstants;
import org.firstinspires.ftc.teamcode.Constantes.LConstants;

@Autonomous(name = "AutoAzulMenor")
public class AutoAzulMenor extends LinearOpMode {
    Follower follower;
    DcMotor motorColetor, motorColetor2, lancador, suporte;

    int etapaAtual = 0;
    int obelisko = 0;
    double voltagem;

    Pose startPose           =   new Pose(64, 10,Math.toRadians(90));
    Pose poseDeLancamento    =   new Pose(48,13,Math.toRadians(114));
    Pose poseDeVisao         =   new Pose(61,15,Math.toRadians(85));
    Pose poseFinal           =   new Pose(48,30,Math.toRadians(90));

    Pose coleta21            =   new Pose(45,34,0);
    Pose coleta21Final       =   new Pose(25,34,0);
    Pose coleta21Ctrl        =   new Pose(48,34);

    Pose coleta22            =   new Pose(40,60,0);
    Pose coleta22Final       =   new Pose(18,60,0);
    Pose coleta22Ctrl        =   new Pose(61,60);

    Pose coleta23            =   new Pose(40,84,0);
    Pose coleta23Final       =   new Pose(18,84,0);
    Pose coleta23Ctrl        =   new Pose(61,84);

    Path pathStart   =  criarLinha(startPose,poseDeLancamento);
    Path pathOlhar   =  criarLinha(poseDeLancamento,poseDeVisao);
    Path pathFinal   =  criarLinha(poseDeLancamento,poseFinal);

    Path path23_1    =  criarCurva(poseDeLancamento,coleta23Ctrl,coleta23);
    Path path23_2    =  criarLinha(coleta23,coleta23Final);
    Path path23_3    =  criarCurva(coleta23Final,coleta23Ctrl,poseDeLancamento);

    Path path22_1    =  criarCurva(poseDeLancamento,coleta22Ctrl,coleta22);
    Path path22_2    =  criarLinha(coleta22,coleta22Final);
    Path path22_3    =  criarCurva(coleta22Final,coleta22Ctrl,poseDeLancamento);

    Path path21_1    =  criarLinha(poseDeLancamento,coleta21);
    Path path21_2    =  criarLinha(coleta21,coleta21Final);
    Path path21_3    =  criarCurva(coleta21Final,coleta21Ctrl,poseDeLancamento);

    @Override
    public void runOpMode() throws InterruptedException {
        boolean truE = true;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorColetor  = hardwareMap.dcMotor.get("motorCO");
        motorColetor2 = hardwareMap.dcMotor.get("motorCO2");
        lancador      = hardwareMap.dcMotor.get("motorLancador");
        suporte       = hardwareMap.dcMotor.get("motorSuporte");
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        suporte.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        suporte.setDirection(DcMotorSimple.Direction.REVERSE);

        pathStart.setLinearHeadingInterpolation(startPose.getHeading(), poseDeLancamento.getHeading());
        pathOlhar.setLinearHeadingInterpolation(poseDeLancamento.getHeading(),poseDeVisao.getHeading());

        path21_1.setLinearHeadingInterpolation(poseDeLancamento.getHeading(),0);
        path21_2.setConstantHeadingInterpolation(0);
        path21_3.setLinearHeadingInterpolation(0, poseDeLancamento.getHeading());

        path22_1.setLinearHeadingInterpolation(poseDeVisao.getHeading(),0);
        path22_2.setConstantHeadingInterpolation(0);
        path22_3.setLinearHeadingInterpolation(0,poseDeLancamento.getHeading());

        path23_1.setLinearHeadingInterpolation(poseDeVisao.getHeading(),0);
        path23_2.setConstantHeadingInterpolation(0);
        path23_3.setLinearHeadingInterpolation(0,poseDeLancamento.getHeading());

        pathFinal.setLinearHeadingInterpolation(poseDeLancamento.getHeading(),poseFinal.getHeading());

        waitForStart();
        while (opModeIsActive() && truE){
            salvarPosicaoDoRobo(follower);
            follower.update();
            switch (etapaAtual){
                case 0:
                    follower.followPath(pathStart);
                    if(follower.atParametricEnd()){
                        voltagem = hardwareMap.voltageSensor.iterator().next().getVoltage();
                        lancarArtefato(calcularPotencia());
                        etapaAtual++;
                        obelisko = 21;
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
                        coletar(false);
                        sleep(500);
                        etapaAtual++;
                    }
                    break;
                case 2:
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
                case 3:
                    if(obelisko == 21){
                        follower.followPath(path21_3);
                        if (follower.atParametricEnd()){
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
            telemetry.update();
        }
    }
    void lancarArtefato(double potencia){
        lancador.setPower(potencia);
        suporte.setPower(potencia);
        sleep(1800);
        motorColetor2.setPower(1);
        motorColetor.setPower(1);
        sleep(500);
        motorColetor2.setPower(0);
        motorColetor.setPower(0);
        sleep(1000);
        motorColetor.setPower(1);
        motorColetor2.setPower(1);
        suporte.setPower(0);
        lancador.setPower(0);
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

            telemetry.addData("Pose a ser salva: ", "X: %.2f , Y: %.2f , Heading: %.2f",xAgora,yAgora,hAgora);
        } catch (IOException e){
            telemetry.addData("Não foi possível salvar a pose do robô","ERROR:" + e.getMessage());
        }
    }
    double calcularPotencia(){
        return 0.78*(13.5/voltagem);
    }
}
