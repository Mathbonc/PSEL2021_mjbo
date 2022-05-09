 #include <QCoreApplication>
#include <QtMath>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

#define X_Goal_blue -0.750
#define X_Goal_yellow 0.750
#define DIST_SHOT 0.3
#define VSSS_ST8_VEL 10
#define VSSS_ANG_VEL 5

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

float Orientation(int quad,float bx, float by, float rx, float ry){
    float desiredOrientation = M_PI,alpha;
    qreal tg=0;

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

std::pair <float,float> calculateShooting_Pos(int quad, float desiredOrientation, float bx, float by){
    std::pair <float,float> shooting_Pos;

    shooting_Pos.first=shooting_Pos.second=0; //Inicializando

    std::cout << "Orienação desejada: " << desiredOrientation << std::endl;

    //Invertendo a orientação
    if(desiredOrientation>0){
        desiredOrientation -= M_PI;
    }else{
        desiredOrientation += M_PI;
    }

    std::cout << "Orienação invertida: " << desiredOrientation << std::endl;

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

    float tg = tan(abs(desiredOrientation));

    y = (DIST_SHOT*sqrt(1+(tg*tg)))/(1+(tg*tg));
    x = tg*y;

    std::cout << x << " " << y << std::endl;

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
        desiredOrientation = Orientation(quad, X_Goal_blue, 0, bola.x(), bola.y());
    }else{
        quad = calculateQuadrant(X_Goal_yellow, 0, bola.x(), bola.y());
        desiredOrientation = Orientation(quad, X_Goal_yellow, 0, bola.x(), bola.y());
    }

    shooting_Pos = calculateShooting_Pos(quad, desiredOrientation, bola.x(), bola.y()); 

    return shooting_Pos;
}

std::pair<float,float> calculate_VSSS_Vel(float desiredOrientation, float dummyOrientation){
    std::pair<float,float> v;//(Left,Right)
    //vw - vira pra direita
    //LW + | RW - : vira pra direita
    v.first=v.second=0;

    if(abs(desiredOrientation - dummyOrientation)<= 0.0872665){
            v.first=v.second=VSSS_ST8_VEL;
    }else if(desiredOrientation > 0){//Bola está em cima
        if(dummyOrientation>0){
            if(dummyOrientation>desiredOrientation){
                v.first = VSSS_ANG_VEL;
                v.second = -VSSS_ANG_VEL;
            }else{
                v.first = -VSSS_ANG_VEL;
                v.second = VSSS_ANG_VEL;
            }
        }else{
            if(dummyOrientation<(-M_PI_2)){//Está no terceiro quad
                v.first = VSSS_ANG_VEL;
                v.second = -VSSS_ANG_VEL;
            }else{
                v.first = -VSSS_ANG_VEL;
                v.second = VSSS_ANG_VEL;
            }
        }
    }else{
        if(dummyOrientation<0){
            if(dummyOrientation>desiredOrientation){
                v.first = VSSS_ANG_VEL;
                v.second = -VSSS_ANG_VEL;
            }else{
                v.first = -VSSS_ANG_VEL;
                v.second = VSSS_ANG_VEL;
            }
        }else{
            if(dummyOrientation<M_PI_2){//Está no primeiro quad
                v.first = VSSS_ANG_VEL;
                v.second = -VSSS_ANG_VEL;
            }else{
                v.first = -VSSS_ANG_VEL;
                v.second = VSSS_ANG_VEL;
            }
        }
    }
    if(abs(desiredOrientation - dummyOrientation)>0.0872665 && abs(desiredOrientation - dummyOrientation)<0.261799){
        v.first /= 3;
        v.second /= 3;
    }

    return v;
}

void VSSS_positioning(Vision *vision, Actuator *actuator, bool isYellow, int playerID){
    fira_message::Ball bola = vision->getLastBallDetection();
    fira_message::Robot roboVision = vision->getLastRobotDetection(isYellow, playerID);

    std::pair<float,float> shooting_Pos;
    std::pair<float,float> v;
    float desiredOrientation;
    int quad;

    shooting_Pos = position_to_shoot(vision,true);

    //printf("Posição esperada: (%f,%f)\n", shooting_Pos.first, shooting_Pos.second);
    //printf("Posição real: (%f,%f)\n", roboVision.x(), roboVision.y());

    quad = calculateQuadrant(shooting_Pos.first,shooting_Pos.second,roboVision.x(),roboVision.y());
    desiredOrientation = Orientation(quad,shooting_Pos.first,shooting_Pos.second,roboVision.x(),roboVision.y());

    v = calculate_VSSS_Vel(desiredOrientation,roboVision.orientation());


    if((roboVision.x()<(shooting_Pos.first+0.01))&&(roboVision.x()>(shooting_Pos.first-0.01))){//Se tiver dentro da área aceitável
        if((roboVision.y()<(shooting_Pos.second+0.01))&&(roboVision.y()>(shooting_Pos.second-0.01))){
            v.first=v.second=0;
        }
    }

    actuator->sendCommand(isYellow,playerID,v.first,v.second);//LW + | RW - : vira pra direita

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

        VSSS_positioning(vision,actuator,true,0);

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
