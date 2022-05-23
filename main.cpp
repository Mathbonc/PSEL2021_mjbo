#include <QCoreApplication>
#include <QtMath>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

#define ANGULAR_V 10
#define XY_VEL 5
#define TOLERABLE_DISTANCE 7.5
#define DIST_SHOT 800
#define BALL_RADIUS 21.5
#define ROBOT_RADIUS 110
#define X_R2_POSITION -1500
#define y_R2_POSITION 0

//Posicionamento
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

float dummyOrientation(float bx, float by, float rx, float ry){
    float desiredOrientation = M_PI,alpha;
    qreal tg;

    int quad = calculateQuadrant(bx,by,rx,ry);

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

std::pair<float,float> shooting_Position(float bx, float by, float rx, float ry, bool pass = false){
    std::pair<float,float> shooting_Pos;
    shooting_Pos.first=shooting_Pos.second=0;

    //sx = Sender.x()
    //sy = Sender.y()
    //rx = Receiver.x()
    //ry = Receiver.y()

    int quad = calculateQuadrant(rx, ry, bx, by);
    float desiredOrientation = dummyOrientation(bx, by, rx, ry);

    //invertendo
    if(!pass){
        if(desiredOrientation>0){
            desiredOrientation -= M_PI;
        }else{
            desiredOrientation += M_PI;
        }
    }

    //Checando se está em algum dos eixos
    if((desiredOrientation < 0.0872665) && (desiredOrientation > -0.0872665)){ //Direita
        shooting_Pos = std::make_pair(bx+0.1,by);
        return shooting_Pos;
    }else if((desiredOrientation < -M_PI + 0.0872665) && (desiredOrientation > M_PI - 0.0872665)){ //Esquerda
        shooting_Pos = std::make_pair(bx-0.1,by);
        return shooting_Pos;
    }else if((desiredOrientation < M_PI_2 + 0.0872665) && (desiredOrientation > M_PI_2 - 0.0872665)){ //Cima
        shooting_Pos = std::make_pair(bx,by+0.1);
        return shooting_Pos;
    }else if((desiredOrientation < -M_PI_2 + 0.0872665) && (desiredOrientation > -M_PI_2 - 0.0872665)){ //Baixo
        shooting_Pos = std::make_pair(bx,by-0.1);
        return shooting_Pos;
    }

    //Posição fora dos eixos
    float x,y;

    float tg=1;

    if(!((by)==0)){
         tg=(abs(rx-bx))/(abs(ry-by));
    }

    y = DIST_SHOT/(sqrt((tg*tg)+1));
    x = tg*y;

    if(quad==3){ //Primeiro
        shooting_Pos.first = bx + x;
        shooting_Pos.second = by + y;
    }else if(quad==4){ //Segundo
        shooting_Pos.first = bx - x;
        shooting_Pos.second = by + y;
    }else if(quad==1){ //Terceiro
        shooting_Pos.first = bx - x;
        shooting_Pos.second = by - y;
    }else if (quad==2){ //Quarto
        shooting_Pos.first = bx + x;
        shooting_Pos.second = by - y;
    }

    return shooting_Pos;
}

std::pair<float,float> receiving_Positon(Vision *vision, float bx, float by, float distance_to_ball){
    SSL_DetectionBall bola;
    bola = vision->getLastBallDetection();

    while((abs(bola.x()-bx) < 10 && abs(bola.y()-by)< 10)){
        vision->processNetworkDatagrams();
        bola = vision->getLastBallDetection();
    }
    // X negativo: Direita | X positivo: esquerda
    // Y negativo: Cima | Y positivo: baixo

    qreal tg = 1;

    if(!((by-bola.y())==0)){
        tg=(abs(bx-bola.x()))/(abs(by-bola.y()));
    }

    //std::cout << alpha << std::endl;

    int quad = 1;

    quad = calculateQuadrant(bola.x(),bola.y(),bx,by);

    //std::cout << quad << std::endl;

    std::pair<float,float> receiving_Pos;
    float x,y;

    y = (distance_to_ball)/(sqrt((tg*tg)+1));
    x = tg*y;

    if(quad==3){
        receiving_Pos.first = bx - x;
        receiving_Pos.second = by - y;
    }else if(quad==4){
        receiving_Pos.first = bx + x;
        receiving_Pos.second = by - y;
    }else if(quad==1){
        receiving_Pos.first = bx + x;
        receiving_Pos.second = by + y;
    }else if (quad==2){
        receiving_Pos.first = bx - x;
        receiving_Pos.second = by + y;
    }

    return receiving_Pos;
}

//Velocidades/Robo
std::pair<float,float> dummyVelCalculator(float desiredOrientation, float dummyOrientation, int quad){
    float absDiff = abs(desiredOrientation-dummyOrientation);
    std::pair<float,float> v;
    float vx,vy; vx=vy=0;
    if((absDiff <= 1.0472)){// bola está na frente
        //std::cout << "Frente" << std::endl;
        vx = XY_VEL; vy=0;
    }else if((absDiff > 1.0472)&&(absDiff < 2.0944)){// bola está nas laterais
        if(quad == 2 || quad == 3){//Esquerda
            if(dummyOrientation>0){
                //std::cout << "Esquerda" << std::endl;
                vy = XY_VEL;
            }else{
                //std::cout << "Direita" << std::endl;
                vy = -XY_VEL;
            }
        }else if(quad == 1 || quad == 4){// Direita
            if(dummyOrientation>0){
                //std::cout << "Direita" << std::endl;
                vy = -XY_VEL;
            }else{
                //std::cout << "Esquerda" << std::endl;
                vy = XY_VEL;
            }
        }
    }else if(absDiff >= 2.0944){ //Bola atrás
        //std::cout << "Tras" << std::endl;
        vx = -XY_VEL; vy=0;
    }
    v.first=vx;
    v.second=vy;
    return v;
}

float dummyAngularVel(float desiredOrientation, float dummy_orientation){
    float vw, Orientation_diff;

    Orientation_diff = abs(desiredOrientation-dummy_orientation);
    vw = abs(ANGULAR_V*(sin(((2*Orientation_diff/(Orientation_diff+M_PI))*M_PI_2))));

    if(Orientation_diff< 0.0349066){
        vw = 0;
    }else if(desiredOrientation > 0){//Bola está em cima
        if(dummy_orientation>0){
            if(dummy_orientation>desiredOrientation){
                vw = -vw;
            }else{
                vw = vw;
            }
        }else{
            if(desiredOrientation < M_PI_2){
                desiredOrientation = -(desiredOrientation+M_PI_2);
            }else{
                desiredOrientation = -(desiredOrientation-M_PI_2);
            }
            if(dummy_orientation<desiredOrientation){//Está no terceiro quad
                vw = -vw;
            }else{
                vw = +vw;
            }
        }
    }else{
        if(dummy_orientation<0){
            if(dummy_orientation>desiredOrientation){
                vw = -vw;
            }else{
                vw = vw;
            }
        }else{
            if(desiredOrientation < - M_PI_2){
                desiredOrientation = -(desiredOrientation+M_PI_2);
            }else{
                desiredOrientation = -(desiredOrientation-M_PI_2);
            }
            if(dummy_orientation<desiredOrientation){//Está no primeiro quad
                vw = -vw;
            }else{
                vw = vw;
            }
        }
    }

    return vw;
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

void dummy_Positioning(Vision *vision, Actuator *actuator, float x_Position, float y_Position,bool isYellow, int playerID, float tolerance){
    SSL_DetectionRobot roboVision = vision->getLastRobotDetection(isYellow, playerID);
    float desiredOrientation;

    //Conseguindo as informações de orientação
    int quad = calculateQuadrant(x_Position,y_Position,roboVision.x(),roboVision.y());
    desiredOrientation = dummyOrientation(x_Position,y_Position,roboVision.x(),roboVision.y());

    //Velocidade angular
    float vw;
    vw = dummyAngularVel(desiredOrientation, roboVision.orientation());

    //Calculando velocidades
    std::pair<float,float> v;
    v = dummyVelCalculator(desiredOrientation,roboVision.orientation(),quad);

    //Enviando
    if(is_Near(x_Position,y_Position,roboVision.x(),roboVision.y(),false,tolerance)){
        actuator->sendCommand(isYellow,playerID,0,0,0);
    }else{
        float x = is_Near(x_Position,y_Position,roboVision.x(),roboVision.y(),true);
        v.first = v.first*(sin(((x/(x+2000))*M_PI_2)));
        v.second = v.second*(sin((x/(x+2000))*M_PI_2));
        actuator->sendCommand(isYellow,playerID,v.first,v.second,vw);
    }
}

void pass_Ball (Vision *vision, Actuator *actuator, bool isYellow, int senderID, int receiverID, bool *r1_pass, bool *r1_r2_pass){
    if(*r1_r2_pass){
        actuator->sendCommand(isYellow,senderID,0,0,0);
        return;
    }
    SSL_DetectionBall bola = vision->getLastBallDetection();
    SSL_DetectionRobot sender = vision->getLastRobotDetection(isYellow,senderID);
    SSL_DetectionRobot receiver = vision->getLastRobotDetection(isYellow, receiverID);

    std::pair<float,float> ShootPos = shooting_Position(bola.x(),bola.y(),X_R2_POSITION,y_R2_POSITION);

    //std::cout << !(is_Near(ShootPos.first,ShootPos.second,sender.x(),sender.y(),false,20)) << " and " << !(*shoot_flag1) << std::endl;

    if(!(is_Near(ShootPos.first,ShootPos.second,sender.x(),sender.y(),false,6)) && !(*r1_pass)){
        dummy_Positioning(vision,actuator,ShootPos.first,ShootPos.second,isYellow,senderID,5);
    }else{
        *r1_pass = true;
        actuator->sendCommand(isYellow,senderID,0,0,0);
    }

    if(*r1_pass){
        float desiredOrientation = dummyOrientation(bola.x(), bola.y(),sender.x(),sender.y());
        float vw = dummyAngularVel(desiredOrientation, sender.orientation());

        if(is_Near(bola.x(),bola.y(),sender.x(),sender.y(),false,(BALL_RADIUS+ROBOT_RADIUS+50))){
            desiredOrientation = dummyOrientation(receiver.x(), receiver.y(),sender.x(),sender.y());
            vw = dummyAngularVel(desiredOrientation, sender.orientation());
        }

        if(abs(desiredOrientation - sender.orientation())> 0.0872665){
            actuator->sendCommand(isYellow,senderID,0,0,vw,false,7);
        }else{
            actuator->sendCommand(isYellow,senderID,5,0,vw,false,7);
            if(is_Near(bola.x(),bola.y(),sender.x(),sender.y(),false,(BALL_RADIUS+ROBOT_RADIUS))){
                (*r1_pass) = false;
                (*r1_r2_pass) = true;
                actuator->sendCommand(isYellow,senderID,0,0,0,false,7);
            }
        }
    }
}

void receive_and_pass(Vision *vision, Actuator *actuator, bool isYellow, int playerID, int receiverID, bool *r1_r2_pass){
    SSL_DetectionBall bola = vision->getLastBallDetection();
    SSL_DetectionRobot roboVision = vision->getLastRobotDetection(isYellow, playerID);
    SSL_DetectionRobot receiver = vision->getLastRobotDetection(isYellow, receiverID);

    float vw,vx,vy;
    float orientation_to_ball, orientation_to_position;
    std::pair<float,float> receive_point; //Necessário saber pra onde a bola ta indo

    orientation_to_ball = dummyOrientation(bola.x(),bola.y(),roboVision.x(),roboVision.y());
    vw = dummyAngularVel(orientation_to_ball,roboVision.orientation()); // ficar de frente com a bola

    //Ir pra posição de receber (de acordo com a formação)
    if(!(*r1_r2_pass)){
        dummy_Positioning(vision,actuator, X_R2_POSITION, y_R2_POSITION ,true,playerID,50   );
    }else{

        receive_point = receiving_Positon(vision,bola.x(),bola.y(),200);// Posição para receber a bola
        orientation_to_position = dummyOrientation(receive_point.first,receive_point.second,roboVision.x(),roboVision.y()); // Orientação para posição

        if(is_Near(roboVision.x(),roboVision.y(),bola.x(),bola.y(),false,(ROBOT_RADIUS + 100))){ // Receber a bola
            actuator->sendCommand(isYellow,playerID,0,0,vw,true,0);
        }else if(is_Near(roboVision.x(),roboVision.y(),bola.x(),bola.y(),false,(ROBOT_RADIUS + 800))){ //ir para a bola
            if(is_Near(bola.x(),bola.y(),roboVision.x(),roboVision.y(),(ROBOT_RADIUS+300))){
                vx = (cos(orientation_to_position)*(XY_VEL/4));
                vy = (sin(orientation_to_position)*(XY_VEL/4));
            }else{
                vx = (cos(orientation_to_position)*(XY_VEL*2));
                vy = (-sin(orientation_to_position)*(XY_VEL*2));
            }

            actuator->sendCommand(isYellow,playerID,vx,vy,0,true);
        }else if(is_Near(roboVision.x(),roboVision.y(),bola.x(),bola.y(),false,(ROBOT_RADIUS + 2500))){ //Vira para direção da bola
            actuator->sendCommand(isYellow,playerID,0,0,vw);
        }
    }
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;
    int role = 0;
    int r1,r2,r3;
    bool r1_shoot = false, r1_r2_pass = false;
    std::pair<int,std::pair<int,int>> roboIDs;

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands//
        vision->processNetworkDatagrams();

        //Conseguindo as funções dos robos
        if(role != 2){
            roboIDs = get_Dummy_role(vision,true,1,2,3);
            r1 = roboIDs.first;
            r2 = roboIDs.second.first;
            r3 = roboIDs.second.second;
            role++;
        }

        if(role==2){
            pass_Ball(vision,actuator,true,r1,r2,&r1_shoot, &r1_r2_pass);
            receive_and_pass(vision,actuator,true,r2,r3,&r1_r2_pass);
            dummy_Positioning(vision,actuator,-4300,2300,true,r3,10);

        }

        // TimePoint//
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
