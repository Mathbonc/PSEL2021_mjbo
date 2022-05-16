#include <QCoreApplication>
#include <QtMath>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

#define ANGULAR_V 2.75
#define TOLERABLE_DISTANCE 7.5

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

float is_Near(float x_goal, float y_goal, float x, float y, bool getDistance = false, float tolerance = 7.5){
    float diff_x, diff_y;

    diff_x = abs(x_goal-x);
    diff_y = abs(y_goal-y);

    if(getDistance){
        diff_x *= diff_x;
        diff_y *= diff_y;
        float hip = sqrt(diff_x+diff_y);

        return hip;
    }

    if(diff_x < tolerance && diff_y < tolerance){
        return 1;
    }else{
        return 0;
    }
}

float dummyAngularVel(float desiredOrientation, float dummy_orientation){
    float vw;

    if(abs(desiredOrientation - dummy_orientation)< 0.0872665){
        vw = 0;
    }else if(desiredOrientation > 0){//Bola está em cima
        if(dummy_orientation>0){
            if(dummy_orientation>desiredOrientation){
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }else{
            if(dummy_orientation<(-M_PI_2)){//Está no terceiro quad
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }
    }else{
        if(dummy_orientation<0){
            if(dummy_orientation>desiredOrientation){
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }else{
            if(dummy_orientation<M_PI_2){//Está no primeiro quad
                vw = -ANGULAR_V;
            }else{
                vw = ANGULAR_V;
            }
        }
    }

    return vw;
}

void dummy_Positioning(Vision *vision, Actuator *actuator, float x_Position, float y_Position,bool isYellow, int playerID, float tolerance){
    SSL_DetectionRobot roboVision = vision->getLastRobotDetection(isYellow, playerID);

    int quad=0;
    float desiredOrientation;

    //Conseguindo as informações de orientação
    quad = calculateQuadrant(x_Position,y_Position,roboVision.x(),roboVision.y());
    desiredOrientation = dummyOrientation(quad,x_Position,y_Position,roboVision.x(),roboVision.y());

    //Velocidade angular
    float vw;
    vw = dummyAngularVel(desiredOrientation, roboVision.orientation());

    //Calculando velocidades
    std::pair<float,float> v;
    v = dummyVelCalculator(desiredOrientation,roboVision.orientation(),quad);

    //Enviando
    if(is_Near(x_Position,y_Position,roboVision.x(),roboVision.y(),false,tolerance)){
        actuator->sendCommand(isYellow,playerID,0,0,0,false,4);
    }else{
        actuator->sendCommand(isYellow,playerID,v.first,v.second,vw,false,4);//X: Frente Y: Esquerda W:Vira esquerda
    }
}

std::pair<int,std::pair<int,int>> get_Dummy_role(Vision *vision, bool isYellow, int playerID1, int playerID2, int playerID3){
    SSL_DetectionBall bola = vision->getLastBallDetection();
    SSL_DetectionRobot ssl1 = vision->getLastRobotDetection(isYellow,playerID1);
    SSL_DetectionRobot ssl2 = vision->getLastRobotDetection(isYellow,playerID2);
    SSL_DetectionRobot ssl3 = vision->getLastRobotDetection(isYellow,playerID3);

    int robo1,robo2,robo3; //Pega a bola || Passa a bola || Faz o gol
    std::pair<int,std::pair<int,int>> RoboIDs;

    //Checando qual robo está mais próximo da bola
    if(is_Near(bola.x(),bola.y(),ssl1.x(),ssl1.y(),true) < is_Near(bola.x(),bola.y(),ssl2.x(),ssl2.y(),true)){ // SSL1 nearest
        if(is_Near(bola.x(),bola.y(),ssl1.x(),ssl1.y(),true) < is_Near(bola.x(),bola.y(),ssl3.x(),ssl3.y(),true)){
            robo1 = playerID1;
            robo2 = playerID2;
            robo3 = playerID3;
        }else{
            robo1 = playerID3;
            robo2 = playerID2;
            robo3 = playerID1;
        }
    }else{ //SSL2 nearest
        if(is_Near(bola.x(),bola.y(),ssl2.x(),ssl2.y(),true) < is_Near(bola.x(),bola.y(),ssl3.x(),ssl3.y(),true)){
            robo1 = playerID2;
            robo2 = playerID3;
            robo3 = playerID1;
        }else{
            robo1 = playerID3;
            robo2 = playerID1;
            robo3 = playerID2;
        }
    }

    RoboIDs.first = robo1;
    RoboIDs.second.first = robo2;
    RoboIDs.second.second = robo3;

    return RoboIDs;
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;
    int role = 0;
    int r1,r2,r3;
    std::pair<int,std::pair<int,int>> roboIDs;

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands//
        vision->processNetworkDatagrams();

        SSL_DetectionBall bola = vision->getLastBallDetection();
        //std::cout << "animal" << std::endl;

        if(role != 2){
            //printf("A\n");
            roboIDs = get_Dummy_role(vision,true,1,2,3);
            r1 = roboIDs.first;
            r2 = roboIDs.second.first;
            r3 = roboIDs.second.second;
            //printf("%d %d %d\n", r1,r2,r3);
            role++;
        }



        // TimePoint//
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
