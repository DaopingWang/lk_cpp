#include <QTimer>
#include <QWidget>
#include <QApplication>

#include "Controller.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "main_state_machine/call_driver.h"
#include "main_state_machine/send_desired_pose.h"
#include "main_state_machine/color_seg_finished.h"
#include "main_state_machine/pose_transfer.h"
#include "main_state_machine/pg_info_transfer.h"

enum State {
    INIT,
    CONNECT,
    WAIT_DS,
    PG_IDENT,
    WAIT_GS,
    SEARCH_CONE,
    GOTO_CONE,
    CATCH_CONE,
    GOTO_GOAL,
    FINISHED,
    PAUSED
};

struct State_machine {
    State current_state;
    State last_state;
    int num_goal;
};

Controller controller;

ros::ServiceClient ab_ident_client;
ros::ServiceClient color_ident_client;
ros::ServiceClient color_seg_client;
ros::ServiceClient from2dto3d_client;
ros::ServiceClient finish_color_seg_client;
ros::ServiceClient driver_command_client;
ros::ServiceClient driver_navi_client;
ros::ServiceClient hockey_catch_client;
ros::ServiceClient driver_pg_info_client;
ros::ServiceClient catch_pg_info_client;

State_machine sm;

std_srvs::Empty empty_srv;
std_srvs::SetBool bool_srv;
main_state_machine::send_desired_pose navi_srv;
main_state_machine::send_desired_pose goal_location_srv;

main_state_machine::call_driver tocone_srv;
main_state_machine::call_driver togoal_srv;
main_state_machine::call_driver exit_srv;

float A = 1.2;
float B = 4;
double ab_ratio;
int color;
State paused_state;


bool ab_ident_finished_callback(main_state_machine::pg_info_transfer::Request& request,
                                main_state_machine::pg_info_transfer::Response& response) {

    ROS_INFO_STREAM("======AB identified======");
    A = request.a;
    B = request.b;
    response.success = true;
    ab_ratio = A / B;
    controller.tellABRatio(ab_ratio);
    color_ident_client.call(empty_srv);
    return true;
}

bool color_ident_finished_callback(main_state_machine::pg_info_transfer::Request& request,
                                   main_state_machine::pg_info_transfer::Response& response){

    ROS_INFO_STREAM("======Color identified " << request.color << "======");
    color = request.color;
    TeamColor c;
    switch (request.color) {
    case 0:
        c = blue;
        controller.tellTeamColor(c);
        break;
    case 1:
        c = yellow;
        controller.tellTeamColor(c);
        break;
    }

    ///// 0 FOR BLUE, 1 FOR YELLOW
    goal_location_srv.request.a = 2.5 * A - 0.55;
    goal_location_srv.request.b = 0;
    goal_location_srv.request.c = 0;

    main_state_machine::pg_info_transfer srv;
    srv.request.a = A;
    srv.request.b = B;
    srv.request.color = color;
    driver_pg_info_client.call(srv);
    catch_pg_info_client.call(srv);

    sm.current_state = WAIT_GS;
    sm.last_state = PG_IDENT;
    response.success = true;
    return true;
}

bool color_seg_finished_callback(main_state_machine::color_seg_finished::Request& request,
                                 main_state_machine::color_seg_finished::Response& response){

    navi_srv.request.a = request.x;
    navi_srv.request.b = request.y;
    navi_srv.request.c = request.theta;

    finish_color_seg_client.call(empty_srv);
    driver_navi_client.call(navi_srv);
    driver_command_client.call(tocone_srv);

    response.success = true;
    sm.current_state = GOTO_CONE;
    sm.last_state = SEARCH_CONE;
    return true;
}

bool driver_tocone_finished_callback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======REACHED CONE LOCATION======");

    hockey_catch_client.call(empty_srv);
    sm.current_state = CATCH_CONE;
    sm.last_state = GOTO_CONE;
    return true;
}

bool driver_togoal_finished_callback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======REACHED GOAL======");


    sm.num_goal += 1;
    controller.reportGoal();
    if (sm.num_goal == 3) {
        sm.current_state = FINISHED;
        sm.last_state = GOTO_GOAL;
    }
    else {
        color_seg_client.call(bool_srv);
        from2dto3d_client.call(empty_srv);
        sm.current_state = SEARCH_CONE;
        sm.last_state = GOTO_GOAL;
    }
    return true;
}

bool target_outbound_callback(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response) {
    ROS_INFO_STREAM("======TARGET OUTBOUND======");
    color_seg_client.call(bool_srv);
    from2dto3d_client.call(empty_srv);
    sm.current_state = SEARCH_CONE;
    sm.last_state = SEARCH_CONE;
    return true;
}

bool driver_exit_finished_callback(std_srvs::Empty::Request& request,
                                   std_srvs::Empty::Response& response) {
    ROS_INFO_STREAM("======EXITED======");

    sm.current_state = SEARCH_CONE;
    sm.last_state = GOTO_GOAL;
    return true;
}

bool hockey_catch_finished_callback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======CONE CAPTURED======");

    switch (sm.num_goal) {
    case 0:
        goal_location_srv.request.b = 0;
        break;
    case 1:
        goal_location_srv.request.b = 0.3;
        break;
    case 2:
        goal_location_srv.request.b = -0.3;
        break;
    }

    driver_navi_client.call(goal_location_srv);
    driver_command_client.call(togoal_srv);
    sm.current_state = GOTO_GOAL;
    sm.last_state = CATCH_CONE;
    return true;
}

bool hockey_catch_failed_callback(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response){
    ROS_INFO_STREAM("======CAPTURE FAILED======");
    color_seg_client.call(bool_srv);
    from2dto3d_client.call(empty_srv);
    sm.current_state = SEARCH_CONE;
    sm.last_state = CATCH_CONE;
    return true;
}

int main(int argc, char **argv)	{
    QApplication app(argc, argv);

    ros::init(argc, argv, "main_state_machine");

    ros::NodeHandle nh;

    ros::ServiceServer ab_ident_finished = nh.advertiseService("ab_ident_finished",
                                                               ab_ident_finished_callback);
    ros::ServiceServer color_ident_finished = nh.advertiseService("color_ident_finished",
                                                                 color_ident_finished_callback);
    ros::ServiceServer color_seg_finished = nh.advertiseService("color_seg_finished",
                                                                                 color_seg_finished_callback);
    ros::ServiceServer driver_tocone_finished = nh.advertiseService("driver_tocone_finished",
                                                                                     driver_tocone_finished_callback);
    ros::ServiceServer driver_togoal_finished = nh.advertiseService("driver_togoal_finished",
                                                                                     driver_togoal_finished_callback);
    ros::ServiceServer target_outbound_srv = nh.advertiseService("target_outbound",
                                                                 target_outbound_callback);
    ros::ServiceServer hockey_catch_finished = nh.advertiseService("catch_finished",
                                                                                    hockey_catch_finished_callback);
    ros::ServiceServer hockey_catch_failed = nh.advertiseService("catch_failed",
                                                                 hockey_catch_failed_callback);

    ros::ServiceServer driver_exit_finished = nh.advertiseService("driver_exit_finished",
                                                                  driver_exit_finished_callback);

    driver_pg_info_client = nh.serviceClient<main_state_machine::pg_info_transfer>("driver_pg_info");

    ab_ident_client = nh.serviceClient<std_srvs::Empty>("start_measure_ab");
    color_ident_client = nh.serviceClient<std_srvs::Empty>("start_measure_color");
    color_seg_client = nh.serviceClient<std_srvs::SetBool>("start_to_color");
    driver_command_client = nh.serviceClient<main_state_machine::call_driver>("call_driver");
    driver_navi_client = nh.serviceClient<main_state_machine::send_desired_pose>("TurtlePose");
    hockey_catch_client = nh.serviceClient<std_srvs::Empty>("catching_state");
    from2dto3d_client = nh.serviceClient<std_srvs::Empty>("from2dto3dwork");
    finish_color_seg_client = nh.serviceClient<std_srvs::Empty>("finish_to_color");
    catch_pg_info_client = nh.serviceClient<main_state_machine::pg_info_transfer>("catch_pg_info");

    ros::ServiceClient pause_driver_client = nh.serviceClient<std_srvs::Empty>("driver_pause");
    ros::ServiceClient pause_catch_client = nh.serviceClient<std_srvs::Empty>("catching_state");

    tocone_srv.request.task = 0;
    togoal_srv.request.task = 1;
    exit_srv.request.task = 2;

    sm.current_state = CONNECT;
    sm.last_state = INIT;
    sm.num_goal = 0;

    ros::Rate rate(100);
    controller.start_alive_timer();
    ROS_INFO_STREAM("======MSM: START CONNECTING======");

    controller.connectToServer("127.0.0.1", 12000);
    while (ros::ok()) {
        app.processEvents();
        if (controller.paused) {
            sm.last_state = sm.current_state;
            sm.current_state = PAUSED;
            controller.paused = false;
        }
        switch (sm.current_state) {

        case CONNECT:
        {
            if (controller.isConnected && !controller.need_reconnect) {
                ROS_INFO_STREAM("======MSM: CONNECTION SUCCESS======");
                controller.reportReady();
                sm.last_state = CONNECT;
                sm.current_state = WAIT_DS;
            } else if (controller.need_reconnect &&!controller.isConnected) {
                ROS_INFO_STREAM("======MSM: CONNECTION FAILED======");
                controller.connectToServer("127.0.0.1", 12000);
            }
            break;
        }
        case WAIT_DS:
        {

            if (controller.de_started) {
                sm.current_state = PG_IDENT;
                sm.last_state = WAIT_DS;
                ab_ident_client.call(empty_srv);
                controller.de_started = false;
                ROS_INFO_STREAM("======MSM: Detection started======");

            }
            break;
        }
        case WAIT_GS:
        {
            if (controller.ab_arrived) {
                A = controller.true_a;
                B = controller.true_b;
                main_state_machine::pg_info_transfer ab_srv;
                ab_srv.request.a = controller.true_a;
                ab_srv.request.b = controller.true_b;
                ab_srv.request.color = color;
                driver_pg_info_client.call(ab_srv);
                goal_location_srv.request.a = 2.5 * A - 0.55;
                goal_location_srv.request.b = 0;
                goal_location_srv.request.c = 0;
                controller.ab_arrived = false;
                ROS_INFO_STREAM("======MSM: ab received======");
            }
            if (controller.wrong_color) {
                ROS_INFO_STREAM("======Color Wrong======");
                main_state_machine::pg_info_transfer color_srv;
                color_srv.request.a = controller.true_a;
                color_srv.request.b = controller.true_b;
                switch (controller.US_COLOR) {
                case yellow:
                    color = 1;
                    break;
                case blue:
                    color = 0;
                    break;
                }
                color_srv.request.color = color;
                catch_pg_info_client.call(color_srv);
                controller.wrong_color = false;
                ROS_INFO_STREAM("======MSM: color received======");
            }
            if (controller.game_started) {
                switch (color) {
                case 0:
                    bool_srv.request.data = false;
                    break;
                case 1:
                    bool_srv.request.data = true;
                    break;
                }
                color_seg_client.call(bool_srv);
                from2dto3d_client.call(empty_srv);
                controller.game_started = false;
                sm.current_state = SEARCH_CONE;
                sm.last_state = WAIT_GS;
                ROS_INFO_STREAM("======MSM: Game started======");

            }
            break;
        }
        case PAUSED:
        {
            ROS_INFO_STREAM("======MSM: paused======");
            switch (sm.last_state) {
            case SEARCH_CONE:{
                finish_color_seg_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case GOTO_CONE: {
                pause_driver_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case CATCH_CONE:{
                pause_catch_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case GOTO_GOAL:{
                pause_driver_client.call(empty_srv);
                paused_state = sm.last_state;
                sm.last_state = PAUSED;
                break;
            }
            case PG_IDENT:{
                ROS_INFO_STREAM("======MSM: Detect-stage pause not implemented======");
                sm.current_state = PG_IDENT;
                sm.last_state = PAUSED;
                break;
            }
            default: {
                if (controller.game_started) {
                    switch (paused_state) {
                    case SEARCH_CONE:
                        color_seg_client.call(bool_srv);
                        from2dto3d_client.call(empty_srv);
                        controller.game_started = false;
                        sm.current_state = SEARCH_CONE;
                        sm.last_state = PAUSED;
                        break;
                    case GOTO_CONE:
                        driver_navi_client.call(navi_srv);
                        driver_command_client.call(tocone_srv);
                        controller.game_started = false;
                        sm.current_state = GOTO_CONE;
                        sm.last_state = PAUSED;
                        break;
                    case GOTO_GOAL:
                        switch (sm.num_goal) {
                        case 0:
                            goal_location_srv.request.b = 0;
                            break;
                        case 1:
                            goal_location_srv.request.b = 0.3;
                            break;
                        case 2:
                            goal_location_srv.request.b = -0.3;
                            break;
                        }

                        driver_navi_client.call(goal_location_srv);
                        driver_command_client.call(togoal_srv);
                        controller.game_started = false;
                        sm.current_state = GOTO_GOAL;
                        sm.last_state = PAUSED;
                        break;
                    case CATCH_CONE:
                        hockey_catch_client.call(empty_srv);
                        controller.game_started = false;
                        sm.current_state = CATCH_CONE;
                        sm.last_state = PAUSED;
                        break;
                    default:
                        break;
                    }
                } else if (controller.de_started) {
                    ROS_INFO_STREAM("======MSM: Detect-stage pause not implemented======");

                }
                break;
            }
            }


            break;
        }
        case FINISHED:
        {
            ROS_INFO_STREAM("======MSM: Done======");
            controller.reportDone();
            break;
        }
        default: {

            break;
        }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return app.exec();
}
