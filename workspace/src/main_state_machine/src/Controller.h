#include <QTimer>
#include <QWidget>
#include <QObject>
#include <QMap>

#include "referee.h"


class Controller:public QObject
{
    Q_OBJECT

    public:
        //Controller(QWidget *parent = 0, int argc = 0, char * argv[] = 0);
    Controller(QObject *parent = 0);
        ~Controller();

        Referee *referee;
    
    TeamColor US_COLOR;
    TeamColor OP_COLOR;
    TeamColor detected_color;
    //int status = 0;

    bool need_reconnect;
    bool isConnected;
    bool de_started;
    bool game_started;
    bool ended;
    bool wrong_color;
    bool ab_arrived;
    bool paused;

    double true_a;
    double true_b;

    void reportReady();
    void changeStatus();
    void tellTeamColor(TeamColor color);
    void tellABRatio(double ratio);
    void reportGoal();
    void reportDone();
    void connectToServer(const QString &ip, int port);
    void start_alive_timer();

    public Q_SLOTS:
    void slotTellAbValue(double ratio);
        void slotTimer();
        void slotSendAlive();
        void slotConnected();
        void slotConnectFailed();
        void slotDisconnected();
        void slotDetectionStart();
        void slotGameStart();
        void slotTrueColorOfTeam(TeamColor);
        void slotAbValues(double a, double b);
        void slotGameOver();
        void slotStopMovement();
};
