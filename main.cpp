#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>

class robotVel{
public:
    float vx;
    float vy;
    float vw;
};

robotVel dummyVelChange(Vision *vision, int playerID,robotVel vector){
    SSL_DetectionBall bola = vision->getLastBallDetection();
    SSL_DetectionRobot roboVision = vision->getLastRobotDetection(true, playerID);

    //std::cout << bola.x() << " and " << roboVision.x() << std::endl;
    //std::cout << roboVision.orientation() << std::endl;

    //X Position
    if(bola.x()<roboVision.x()){//Direita da bola
        vector.vx = 1.0;
    }else if(bola.x()>roboVision.x()){//Esquerda da bola
        vector.vx = -1.0;
    }else {//Igual com a bola
        vector.vx = 0.0;
    }

    //Y Position
    if(bola.y()<roboVision.y()){//Acima da bola
        vector.vy = 1.0;
    }else if(bola.y()>roboVision.y()){//Abaixo da bola
        vector.vy = -1.0;
    }else {//Igual com a bola
        vector.vy = 0.0;
    }

    return vector;
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    int playerID = 3;
    robotVel vector;

    // Desired frequency
    int desiredFrequency = 60;

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        vector = dummyVelChange(vision,playerID,vector);
        //actuator->sendCommand(true, 0, 0.0, 0.0, 0.25,false,3.0,true);
        //actuator->sendCommand(true, playerID, vector.vx, vector.vy, 0.0,false,3.0,true);//X: Frente Y: Esquerda W:Vira esquerda

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
