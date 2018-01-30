//**************************************//
//****************

// COUT PROBLEM

//**********************//
///**************************************/

#include <iostream>
#include <cstdlib>
#include "Controller.h"

//Referee * referee;

//Controller::Controller(QWidget * parent, int argc, char * argv[]) :
//	argc(argc),
//	argv(argv) {


Controller::Controller(QObject * parent) :QObject(parent),referee(0){

        referee = new Referee(3, this);

    // listen signals
    connect(referee, SIGNAL(disconnected()), this, SLOT(slotDisconnected()));
    connect(referee, SIGNAL(detectionStart()), this, SLOT(slotDetectionStart()));
    connect(referee, SIGNAL(gameStart()), this, SLOT(slotGameStart()));
    connect(referee, SIGNAL(trueColorOfTeam(TeamColor)), this, SLOT(slotTrueColorOfTeam(TeamColor)));
    connect(referee, SIGNAL(gameOver()), this, SLOT(slotGameOver()));
    connect(referee, SIGNAL(stopMovement()), this, SLOT(slotStopMovement()));
    connect(referee, SIGNAL(connected()), this, SLOT(slotConnected()));
    connect(referee, SIGNAL(connectFailed()), this, SLOT(slotConnectFailed()));
    connect(referee, SIGNAL(abValues(double,double)), this, SLOT(slotAbValues(double,double)));


    isConnected = false;
    need_reconnect = false;
    wrong_color = false;
    ab_arrived = false;
    de_started = false;
    game_started = false;
    ended = false;
    paused = false;

/*
        double ratio = 0.6;
        if(de_started){

          referee->tellAbRatio(ratio);
          referee->tellTeamColor(teamcolor);
        }*/
}

Controller::~Controller() {

    delete referee;

}

//void Controller::changeStatus() {

//	status = status + 1;

//}


/**********************************************************/
/****************      methods            *****************/
/****************                         *****************/
/**********************************************************/

void Controller::reportReady(){

    referee->reportReady();

}

void Controller::start_alive_timer(){
    QTimer *aliveTimer = new QTimer(this);
    connect(aliveTimer, SIGNAL(timeout()), this, SLOT(slotSendAlive()));
    aliveTimer->start(15000);
}

void Controller::slotTellAbValue(double ratio)
{

    referee->tellAbRatio(ratio);
}

void Controller::tellTeamColor(TeamColor color)
{
    //Teamcolor colors = color;
    detected_color = color;
    referee->tellTeamColor(color);
}

void Controller::tellABRatio(double ratio)
{
    referee->tellAbRatio(ratio);
}

void Controller::reportGoal()
{
    referee->reportGoal();
}

void Controller::reportDone()
{
    referee->reportDone();
}

void Controller::connectToServer(const QString &ip, int port)
{
    std::cout << "Connecting to server..." << std::endl;
    referee->connectToServer(ip, port);
}

void Controller::slotSendAlive()
{
      std::cout << "send alive, send alive" << std::endl;
      referee->sendAlive();
}


/**********************************************************/
/*******************   signals      ***********************/
/*******************                ***********************/
/**********************************************************/
void Controller::slotTimer() {

    //std::cout << "Current tick: " << tick << std::endl;


    // referee->reportReady();
    // referee->reportDone();
    // referee->reportGoal();
    // referee->tellEgoPos(double x, double y);

    //switch(status) {

        //STATUS_LIST(MAKE_CASE)

        //	break;
    //	default:

    //		break;
    //}


    //if (ended) {
    //	timer->stop();
    //	app->quit();
    //}

    //tick += 1;
}


//! Wird gesendet, falls die Verbindung zum Server erfolgreich war.
void Controller::slotConnected() {

    isConnected = true;
    need_reconnect = false;

}

void Controller::slotConnectFailed() {

    std::cout << ("Connection failed.") << std::endl;
    isConnected = false;
    need_reconnect = true;
}

void Controller::slotDisconnected() {

    //std::cerr << BOLD(FRED("Server Lost")) << std::endl;
    paused = true;
}

void Controller::slotDetectionStart() {

    de_started = true;
    game_started = false;
    paused = false;
    //changeStatus(status_free_explore);

}

void Controller::slotGameStart() {

     game_started = true;
     de_started = false;
     paused = false;
    //changeStatus(status_select_target);

}

void Controller::slotTrueColorOfTeam(TeamColor color) {

    if (color == blue) {

        US_COLOR = blue;
        OP_COLOR = yellow;

        //cout << FBLU("We are now blue.") << endl;
        std::cout << "We are now blue" << std::endl;
    } else {

        US_COLOR = yellow;
        OP_COLOR = blue;

        //cout << FYEL("We are yellow.") << endl;
        std::cout << "We are yellow" << std::endl;
    }
    if (detected_color != color) {
        wrong_color = true;
    }

}

void Controller::slotAbValues(double a, double b)
{
    ab_arrived = true;
    true_a = a;
    true_b = b;
}

void Controller::slotGameOver() {
    ended = true;
}

void Controller::slotStopMovement() {
    paused = true;
}



