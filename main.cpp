#include <QCoreApplication>
#include <QtMath>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

#define X_GOAL_BLUE -0.750
#define X_GOAL_YELLOW 0.750
#define X_BLUE_KEEPER 0.700
#define X_YELLOW_KEEPER -0.700
#define BLUE_FIELD_2 0.375
#define YELLOW_FIELD_2 -0.375
#define DIST_SHOT 0.35
#define DIST_RECEIVE 0.25
#define VSSS_ST8_VEL 1.5
#define VSSS_ANG_VEL 10

//Suport Functions

float is_Near(float x_goal, float y_goal, float x, float y, bool getDistance = false, float tolerance = 0.01){
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

float invert_Orientation(float orientation){
    if(orientation>0){
        orientation-=M_PI;
    }else{
        orientation+=M_PI;
    }
    return orientation;
}

std::pair <float,float> calculateShooting_Pos(int quad, float orientation_to_objective, float gx, float bx, float by){
    std::pair <float,float> shooting_Pos;

    shooting_Pos.first=shooting_Pos.second=0; //Inicializando

    //std::cout << "Orienação desejada: " << desiredOrientation << std::endl;

    //Invertendo a orientação
    if(orientation_to_objective>0){
        orientation_to_objective -= M_PI;
    }else{
        orientation_to_objective += M_PI;
    }

    //std::cout << "Orienação invertida: " << desiredOrientation << std::endl;

    //Checando se está em algum do eixos
    if((orientation_to_objective < 0.0872665) && (orientation_to_objective > -0.0872665)){ //Direita
        shooting_Pos = std::make_pair(bx+0.1,by);
        return shooting_Pos;
    }else if((orientation_to_objective < -M_PI + 0.0872665) && (orientation_to_objective > M_PI - 0.0872665)){ //Esquerda
        shooting_Pos = std::make_pair(bx-0.1,by);
        return shooting_Pos;
    }else if((orientation_to_objective < M_PI_2 + 0.0872665) && (orientation_to_objective > M_PI_2 - 0.0872665)){ //Cima
        shooting_Pos = std::make_pair(bx,by+0.1);
        return shooting_Pos;
    }else if((orientation_to_objective < -M_PI_2 + 0.0872665) && (orientation_to_objective > -M_PI_2 - 0.0872665)){ //Baixo
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

std::pair <float,float> ball_Predictor(Vision *vision, bool isYellow){
    fira_message::Ball bola = vision->getLastBallDetection();
    std::pair<float,float> receiving_Pos;
    double tg = 1,y,by;

    if(bola.vx() != 0 || bola.vy() != 0){
        tg = abs(bola.vx())/abs(bola.vy());
    }

    y = (DIST_RECEIVE)/(sqrt((tg*tg)+1));

    by = bola.y();

    if(isYellow){
        receiving_Pos.first = X_GOAL_BLUE + 0.100;
        if(bola.vx() < 0 && bola.vy() > 0){ //Indo para o 2Q
            receiving_Pos.second = by + y;
        }else if(bola.vx() < 0 && bola.vy() < 0){ //Indo para o 3Q
            receiving_Pos.second = by - y;
        }
    }else{
        receiving_Pos.first = X_GOAL_YELLOW - 0.100;
        if(bola.vx() > 0 && bola.vy() > 0){ //Indo para o 1Q
            receiving_Pos.second = by + y;
        }else if (bola.vx() > 0 && bola.vy() < 0){ //Indo para o 4Q
            receiving_Pos.second = by - y;
        }
    }

    return receiving_Pos;
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

std::pair<double,double> VSSS_Velocity(float desiredOrientation, float robotOrientation, float distance, float vel_multiplier = 1,bool rotate_Only = false){
    float L = 0.075;
    float r = 0.0325;
    double vw;
    std::pair<double,double> v;

    //Calcular velocidade linear
    float V = (VSSS_ST8_VEL*vel_multiplier)*(sin(((distance/(distance+0.650))*M_PI_2)));

    if(abs(desiredOrientation-robotOrientation) > (M_PI_2 - 0.0872665) && abs(desiredOrientation-robotOrientation) < (M_PI_2 + 0.0872665) ){//Caso a bola esteja na lateral do robô
        V = 0;//Rotaciona apenas
        if(abs(desiredOrientation-robotOrientation) > (M_PI_2)){ // ROtaciona pro lado oposto caso esteja mais pra trás do que pra frente
            robotOrientation = invert_Orientation(robotOrientation);
        }
    }else if(abs(desiredOrientation-robotOrientation) > (M_PI_2 + 0.0872665)){ //A bola está atrás do robo
        V = -V; //Usa a ré
        robotOrientation = invert_Orientation(robotOrientation);
    }

    //Calcular velociade angular
    vw = VSSS_Angular_Velocity(desiredOrientation,robotOrientation); 
    vw *= 2*vel_multiplier;

    if(rotate_Only){
        V=0;
    }



    v.first = ((2*V)-(L*vw))/(2*r);
    v.second = ((2*V)+(L*vw))/(2*r);

    return v;
}

void VSSS_positioning(Vision *vision, Actuator *actuator, bool isYellow, int playerID, float x_position, float y_position, float vel_multiplier = 1,bool rotate_Only = false, float tolerance = 0.01){
    fira_message::Robot roboVision = vision->getLastRobotDetection(isYellow, playerID);

    std::pair<double,double> v;
    float desiredOrientation;

    desiredOrientation = Orientation(x_position,y_position,roboVision.x(),roboVision.y());

    if(is_Near(x_position,y_position,roboVision.x(),roboVision.y(),false,tolerance) || rotate_Only){
        if(abs(abs(desiredOrientation) - abs(roboVision.orientation()))<= 0.0872665){
            actuator->sendCommand(isYellow,playerID,0,0);
        }else{
            v = VSSS_Velocity(desiredOrientation,roboVision.orientation(),is_Near(x_position,y_position,roboVision.x(),roboVision.y(),true),vel_multiplier,true);
            actuator->sendCommand(isYellow,playerID,v.first,v.second);
        }
    }else{
        v = VSSS_Velocity(desiredOrientation,roboVision.orientation(),is_Near(x_position,y_position,roboVision.x(),roboVision.y(),true),vel_multiplier);
        actuator->sendCommand(isYellow,playerID,v.first,v.second);//LW + | RW - : vira pra direita
    }
}

void shoot_Ball(Vision *vision, Actuator *actuator, bool isYellow, int playerID, bool *Shot){

    fira_message::Ball bola = vision->getLastBallDetection();
    fira_message::Robot roboVision = vision->getLastRobotDetection(isYellow,playerID);

    float desiredOrientation = Orientation(bola.x(),bola.y(),roboVision.x(),roboVision.y());

    double abs_DO = abs(desiredOrientation);
    double abs_RO = abs(roboVision.orientation());

    std::pair<float,float> v;
    float vw;

    vw = VSSS_Angular_Velocity(desiredOrientation,roboVision.orientation());

    v.first = (-(0.075*vw))/(2*0.0325);
    v.second = ((0.075*vw))/(2*0.0325);

    if(abs(abs_DO-abs_RO)>0.0523599){
        actuator->sendCommand(isYellow,playerID,v.first,v.second);
    }else{
        float b_vx,b_vy;
        while(!*(Shot)){
            vision->processNetworkDatagrams();
            bola = vision->getLastBallDetection();

            b_vx = abs(bola.vx());
            b_vy = abs(bola.vy());

            b_vx = abs(b_vx-0.03);
            b_vy = abs(b_vy-0.03);

            if(b_vx > 0.05 || b_vy > 0.05){
                *(Shot)=true;
            }else{
                actuator->sendCommand(isYellow,playerID,200,200);
                //if(abs(desiredOrientation-roboVision.orientation())<0.0523599){

                //}
                /*else{
                    actuator->sendCommand(isYellow,playerID,-200,-200);
                }
                */
            }
        }
    }
}

void shoot_to(Vision *vision, Actuator *actuator,  bool isYellow, int playerID, float x_goal, float y_goal,bool *Shot){
    if(*(Shot)){
        actuator->sendCommand(isYellow,playerID,0,0);
        return;
    }
    fira_message::Ball bola = vision->getLastBallDetection();
    fira_message::Robot roboVision = vision->getLastRobotDetection(isYellow,playerID);

    int quad = calculateQuadrant(x_goal,y_goal,bola.x(),bola.y());
    float orientation_to_goal = Orientation(x_goal,y_goal,bola.x(),bola.y());
    std::pair<float,float> shootingPosition = calculateShooting_Pos(quad,orientation_to_goal,x_goal,bola.x(),bola.y());

    if(is_Near(shootingPosition.first,shootingPosition.second,roboVision.x(),roboVision.y()) && !*(Shot)){
        shoot_Ball(vision,actuator,isYellow,playerID,Shot);
    }else if(*(Shot)){
        actuator->sendCommand(isYellow,playerID,0,0);
    }else{
        float desiredOrientation = Orientation(shootingPosition.first,shootingPosition.second,roboVision.x(),roboVision.y());
        std::pair<double,double> v = VSSS_Velocity(desiredOrientation,roboVision.orientation(),is_Near(shootingPosition.first,shootingPosition.second,roboVision.x(),roboVision.y(),true));
        actuator->sendCommand(isYellow,playerID,v.first,v.second);
    }
}

std::pair<float,float> keeper_Velocity(Vision *vision, bool isYellow, int playerID,float x_position, float y_position){
    fira_message::Ball bola = vision->getLastBallDetection();
    fira_message::Robot robo = vision->getLastRobotDetection(isYellow,playerID);
    float desiredOrientation;
    float vx,vy,vw,robotOrientation = 0;
    std::pair<float,float> v;

    if(abs(desiredOrientation-robotOrientation) > (M_PI_2)){ //A bola está atrás do robo
        robotOrientation = invert_Orientation(robo.orientation());
    }else{
        robotOrientation = robo.orientation();
    }

    if(is_Near(bola.x(),bola.y(),robo.x(),robo.y(),false,0.1)){ // Bola próxima, o alvo se torna bola;
        desiredOrientation = Orientation(bola.x(),bola.y(),robo.x(),robo.y());
    }else{
        desiredOrientation = Orientation(x_position,y_position,robo.x(),robo.y());
    }

    vw = 2*VSSS_Angular_Velocity(desiredOrientation,robotOrientation);

    vx = (-(0.075*vw))/(2*0.0325);
    vy= ((0.075*vw))/(2*0.0325);

    if(isYellow){
        if(bola.x()> YELLOW_FIELD_2/1.5){//rotaciona pra posição
            v.first = vx;
            v.second = vy;
            if(abs(abs(desiredOrientation) - abs(robotOrientation))>0.0872665){
                return v;
            }else{
                return std::make_pair(0,0);
            }
        }else{ //Vai pra posição de interceptação
            v = VSSS_Velocity(desiredOrientation,robotOrientation,is_Near(x_position,y_position,robo.x(),robo.y(),true),3);
            if(is_Near(bola.x(),bola.y(),robo.x(),robo.y())){
                return std::make_pair(0,0);
            }else{
                return v;
            }
        }
    }else{
        if(bola.x() < BLUE_FIELD_2 / 1.5){//rotaciona pra posição
            v.first = vx;
            v.second = vy;
            if(abs(abs(desiredOrientation) - abs(robotOrientation))>0.0872665){
                return v;
            }else{
                return std::make_pair(0,0);
            }
        }else{//Vai pra posição de interceptação
            v = VSSS_Velocity(desiredOrientation,robotOrientation,is_Near(x_position,y_position,robo.x(),robo.y(),true),3);
            if(is_Near(x_position,y_position,robo.x(),robo.y())){
                return std::make_pair(0,0);
            }else{
                return v;
            }
        }
    }
}

void keeper(Vision *vision, Actuator *actuator, bool isYellow, int playerID){
    fira_message::Ball bola = vision->getLastBallDetection();
    fira_message::Robot roboVision = vision->getLastRobotDetection(isYellow,playerID);

    std::pair<float,float> receiving_Pos,v;

    if(isYellow){
        if(bola.x() > 0){ // Fica na posição de goleiro
            VSSS_positioning(vision,actuator,isYellow,playerID,X_YELLOW_KEEPER,0);
        }else{
            receiving_Pos = ball_Predictor(vision,isYellow);
            if(is_Near(bola.x(),bola.y(),roboVision.x(),roboVision.y(),false,0.001)){ // Se a bola estiver perto, ficar parado
                actuator->sendCommand(isYellow,playerID,0,0);
            }else{
                v=keeper_Velocity(vision,isYellow,playerID,receiving_Pos.first,receiving_Pos.second);
                actuator->sendCommand(isYellow,playerID,v.first,v.second);
            }
        }
    }else{
        if(bola.x() < 0){ // Fica na posição de goleiro
            VSSS_positioning(vision,actuator,isYellow,playerID,X_BLUE_KEEPER,0);
        }else{
            receiving_Pos = ball_Predictor(vision,isYellow);
            if(is_Near(bola.x(),bola.y(),roboVision.x(),roboVision.y(),false,0.001)){ // Se a bola estiver perto, ficar parado
                actuator->sendCommand(isYellow,playerID,0,0);
            }else{
                v=keeper_Velocity(vision,isYellow,playerID,receiving_Pos.first,receiving_Pos.second);
                actuator->sendCommand(isYellow,playerID,v.first,v.second);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;
    bool Shot=false;
    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        fira_message::Ball bola = vision->getLastBallDetection();
        shoot_to(vision,actuator,false,0,X_GOAL_BLUE,0.5,&Shot);
        keeper(vision,actuator,true,0);
        //VSSS_positioning(vision,actuator,true,0,bola.x(),bola.y(),3);


        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
