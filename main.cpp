#include <QCoreApplication>
#include <QtMath>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

#define ANGULAR_V 2.75

int calculateQuadrant(float bx, float by, float rx, float ry){
    int quad=0;
    if((bx>rx)&&(by>ry)){//Primeiro Quadrante
        quad = 1;
    }else if((bx<rx)&&(by>ry)){//Segundo Quadrante
        quad = 2;
    }else if((bx<rx)&&(by<ry)){//Terceiro Quadrante
        quad = 3;
    }else if((bx>rx)&&(by<ry)){//Quarto Quadrante
        quad = 4;
    }else if((abs(bx-rx)<2)&&(by>ry)){//Diretamente em cima
        quad = 11;
    }else if((abs(by-ry)<2)&&(bx<rx)){//Diretamene na esquerda
        quad = 12;
    }else if((abs(bx-rx)<2)&&(by<ry)){//Diretamene em baixo
        quad = 13;
    }else if((abs(by-ry)<2)&&(by>ry)){//Diretament na direita
        quad = 14;
    }
    return quad;
}

float dummyOrientation(int quad,float bx, float by, float rx, float ry){
    float desiredOrientation = M_PI,alpha;
    qreal tg;

    if(!((by-ry)==0)){
        tg=(abs(bx-rx))/(abs(by-ry));
    }
    alpha = qAtan(tg);// Temos o alpha em radianos

    switch (quad) {
    case 1:
        desiredOrientation = (M_PI_2 - alpha);
        break;
    case 2:
        desiredOrientation = (M_PI_2 + alpha);
        break;
    case 3:
        desiredOrientation = ((-M_PI_2) - alpha);
        break;
    case 4:
        desiredOrientation = ((-M_PI_2) + alpha);
        break;
    case 11://Cima
        desiredOrientation = M_PI_2;
        break;
    case 12://Esquerda
        desiredOrientation = -M_PI;
        break;
    case 13://Baixo
        desiredOrientation = -M_PI_2;
        break;
    case 14://Direita
        desiredOrientation = 0.0;
        break;
    default:
        break;
    }

    return desiredOrientation;
}

std::pair<float,float> dummyVelCalculator(float desiredOrientation, float dummyOrientation, int quad){
    float absDiff = abs(desiredOrientation-dummyOrientation);
    std::pair<float,float> v;
    float vx,vy; vx=vy=0;
    if((absDiff <= 1.0472)){// bola está na frente
        //std::cout << "Frente" << std::endl;
        vx = 1.5; vy=0;
    }else if((absDiff > 1.0472)&&(absDiff < 2.0944)){// bola está nas laterais
        if(quad == 2 || quad == 3){//Esquerda
            if(dummyOrientation>0){
                //std::cout << "Esquerda" << std::endl;
                vy = 1.5;
            }else{
                //std::cout << "Direita" << std::endl;
                vy = -1.5;
            }
        }else if(quad == 1 || quad == 4){// Direita
            if(dummyOrientation>0){
                //std::cout << "Direita" << std::endl;
                vy = -1.5;
            }else{
                //std::cout << "Esquerda" << std::endl;
                vy = 1.5;
            }
        }
    }else if(absDiff >= 2.0944){ //Bola atrás
        //std::cout << "Tras" << std::endl;
        vx = -1.5; vy=0;
    }
    v.first=vx;
    v.second=vy;
    return v;
}

void dummyControl(Vision *vision, Actuator *actuator, bool isYellow, int playerID){
    SSL_DetectionBall bola = vision->getLastBallDetection();
    SSL_DetectionRobot roboVision = vision->getLastRobotDetection(isYellow, playerID);
    int quad=0;
    float desiredOrientation,vw;

    //Conseguindo as informações de orientação
    quad = calculateQuadrant(bola.x(),bola.y(),roboVision.x(),roboVision.y());
    desiredOrientation = dummyOrientation(quad,bola.x(),bola.y(),roboVision.x(),roboVision.y());

    if(abs(desiredOrientation - roboVision.orientation())< 0.0872665){
        vw = 0;
    }else if(desiredOrientation > 0){//Bola está em cima
        if(roboVision.orientation()>0){
            if(roboVision.orientation()>desiredOrientation){
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }else{
            if(roboVision.orientation()<(-M_PI_2)){//Está no terceiro quad
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }
    }else{
        if(roboVision.orientation()<0){
            if(roboVision.orientation()>desiredOrientation){
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }else{
            if(roboVision.orientation()<M_PI_2){//Está no primeiro quad
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }
    }

    //Calculando velocidades
    std::pair<float,float> v;
    v = dummyVelCalculator(desiredOrientation,roboVision.orientation(),quad);

    actuator->sendCommand(isYellow,playerID,v.first,v.second,vw,false,4);//X: Frente Y: Esquerda W:Vira esquerda
    //printf("Quad: %d | desiredOrientation: %f | dummyOrientation: %f\n", quad, desiredOrientation, roboVision.orientation());
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands//
        vision->processNetworkDatagrams();

        dummyControl(vision,actuator, true,1);
        //dummyControl(vision,actuator, false,1);
        //dummyControl(vision,actuator, true,2);
        //dummyControl(vision,actuator, false,2);


        // TimePoint//
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
