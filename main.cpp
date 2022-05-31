#include <QCoreApplication>
#include <QtMath>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

#define X_Goal_blue -0.750
#define X_Goal_yellow 0.750
#define DIST_SHOT 0.15
#define VSSS_ST8_VEL 1.5
#define VSSS_ANG_VEL 10

//Suport Functions

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

float Orientation(float bx, float by, float rx, float ry){
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

std::pair <float,float> calculateShooting_Pos(int quad, float desiredOrientation, float gx, float bx, float by){
    std::pair <float,float> shooting_Pos;

    shooting_Pos.first=shooting_Pos.second=0; //Inicializando

    //std::cout << "Orienação desejada: " << desiredOrientation << std::endl;

    //Invertendo a orientação
    if(desiredOrientation>0){
        desiredOrientation -= M_PI;
    }else{
        desiredOrientation += M_PI;
    }

    //std::cout << "Orienação invertida: " << desiredOrientation << std::endl;

    //Checando se está em algum do eixos
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

    //Caso não esteja nos eixos, fazer o cálculo usando tg obtida
    float x,y;

    float tg=1;

    if(!((by)==0)){
         tg=(abs(gx-bx))/(abs(by));
    }

    y = DIST_SHOT/(sqrt((tg*tg)+1));
    x = tg*y;

    //std::cout << "TG obtida: " << tg << std::endl;
    //std::cout << x << " " << y << std::endl;

    //Aqui, os quadrantes são "invertidos", então se quad é 3, na verdade a posição está no inverso, que seria o primeiro
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

std::pair<float,float> position_to_shoot(Vision *vision, bool isYellow){
    fira_message::Ball bola = vision->getLastBallDetection();
    fira_message::Field gol = vision->getLastGeometryData();
    //fira_message::Robot robo = vision->getLastRobotDetection();

    int quad=0;
    float desiredOrientation;
    std::pair <float,float> shooting_Pos;

    if(isYellow){
        quad = calculateQuadrant(X_Goal_blue, 0, bola.x(), bola.y());
        desiredOrientation = Orientation(X_Goal_blue, 0, bola.x(), bola.y());
        shooting_Pos = calculateShooting_Pos(quad, desiredOrientation, X_Goal_blue,bola.x(), bola.y());
    }else{
        quad = calculateQuadrant(X_Goal_yellow, 0, bola.x(), bola.y());
        desiredOrientation = Orientation(X_Goal_yellow, 0, bola.x(), bola.y());
        shooting_Pos = calculateShooting_Pos(quad, desiredOrientation, X_Goal_yellow,bola.x(), bola.y());
    }

    return shooting_Pos;
}

//Robot Functions

double VSSS_Angular_Velocity(float desiredOrientation, float dummy_orientation){
    float Orientation_diff;
    double vw;

    Orientation_diff = abs(desiredOrientation-dummy_orientation);
       vw = abs(VSSS_ANG_VEL*(sin(((2*Orientation_diff/(Orientation_diff+M_PI))*M_PI_2))));

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

std::pair<double,double> VSSS_Velocity(float desiredOrientation, float robotOrientation, float distance, bool rotate_Only = false){
    float L = 0.075;
    float r = 0.0325;
    double vw;
    std::pair<double,double> v;

    //Calcular velociade angular
    vw = VSSS_Angular_Velocity(desiredOrientation,robotOrientation);
    //Calcular velocidade linear
    float V = VSSS_ST8_VEL*(sin(((distance/(distance+0.650))*M_PI_2)));

    if(rotate_Only){
        V=0;
    }

    if(abs(desiredOrientation-robotOrientation)>M_PI_2){//3 point turn
        V = -V; //Sempre dar ré
    }

    v.first = ((2*V)-(L*vw))/(2*r);
    v.second = ((2*V)+(L*vw))/(2*r);

    //std::cout << "ANGULAR: " << vw << std::endl;
    //std::cout << "LINEAR 1: " << v.first << std::endl;
    //std::cout << "LINEAR 2: " << v.second << std::endl;

    return v;
}

void VSSS_positioning(Vision *vision, Actuator *actuator, bool isYellow, int playerID, float x_position, float y_position, bool rotate_Only = false, float tolerance = 0.01){
    fira_message::Robot roboVision = vision->getLastRobotDetection(isYellow, playerID);

    std::pair<double,double> v;
    float desiredOrientation;

    desiredOrientation = Orientation(x_position,y_position,roboVision.x(),roboVision.y());

    if(is_Near(x_position,y_position,roboVision.x(),roboVision.y(),false,tolerance) || rotate_Only){
        if(abs(desiredOrientation - roboVision.orientation())<= 0.0872665){
            actuator->sendCommand(isYellow,playerID,0,0);
        }else{
            v = VSSS_Velocity(desiredOrientation,roboVision.orientation(),is_Near(x_position,y_position,roboVision.x(),roboVision.y(),true),true);
            actuator->sendCommand(isYellow,playerID,v.first,v.second);
        }
    }else{
        v = VSSS_Velocity(desiredOrientation,roboVision.orientation(),is_Near(x_position,y_position,roboVision.x(),roboVision.y(),true));
        actuator->sendCommand(isYellow,playerID,v.first,v.second);//LW + | RW - : vira pra direita
    }
}

void ball_Shooting(Vision *vision, Actuator *Actuator, bool isYellow, int playerID, bool *isShoot){
    fira_message::Ball bola = vision->getLastBallDetection();

    float b_vx,b_vy;
    b_vx = abs(bola.vx());
    b_vy = abs(bola.vy());

    //A bola terá que ter mais que 0.05 de velocidade para ser considerada chutada.
    b_vx = abs(b_vx-0.03);
    b_vy = abs(b_vy-0.03);

    //std::cout << b_vx <<" And " << b_vy << std::endl;
    if(b_vx > 0.05 || b_vy > 0.05){
        *(isShoot)=false;
    }else{
        Actuator->sendCommand(isYellow,playerID,VSSS_ST8_VEL*2,VSSS_ST8_VEL*2);
    }
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        fira_message::Ball bola = vision->getLastBallDetection();
        VSSS_positioning(vision,actuator,true,0,bola.x(),bola.y());

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
