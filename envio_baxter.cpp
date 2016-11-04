// Control
// Executing
// ./baxter.sh sim
// roslaunch baxter_gazebo baxter_world.launch
// rosrun baxter_tools enable_robot.py -e
// rosrun baxter..AA
////////////////// Executing
// /home/jose/baxter-workspace/src/baxter_cpp/launch
// roslaunch baxter_cpp l_baxter_cpp.launch
// rosrun baxter_cpp OSIK_baxter
#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h> 
#include <robot-model/robot-model.hpp>
#include <tf2_msgs/TFMessage.h>
#include <osik-control/math-tools.hpp>		
#include <osik-control/kine-task.hpp>
#include <osik-control/kine-task-pose.hpp>
#include <osik-control/kine-solver-WQP.hpp>
#include <baxter_cpp/robotSpecifics.h>
#include <baxter_cpp/tools.hpp>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <math.h>
#include <stdlib.h>
#include <cmath>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <baxter_cpp/markers.hpp>
#include <tf/transform_datatypes.h>
#include <fstream> 
#include <unistd.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>

//#include <bits/stdc++.h>
using namespace osik;
using namespace std;

class MouseData
{
public:
  MouseData()
    : msg_(new geometry_msgs::Vector3)
  {
     std::cout << "Mouse: " <<msg_->x << std::endl;
  }
  void readMouseData(const geometry_msgs::Vector3::ConstPtr& msg)
  {
    msg_ = msg;
  }
  geometry_msgs::Vector3::ConstPtr getData()
  {
    return msg_;
  }
private:
  geometry_msgs::Vector3::ConstPtr msg_;
};


class EndPoint
{
public:
  EndPoint()
    : msg_(new baxter_core_msgs::EndpointState)
  {
     std::cout << "EndPoint" <<msg_->pose.position.x << std::endl;
  }
  void readEndPoint(const baxter_core_msgs::EndpointState::ConstPtr& msg)
  {
    msg_ = msg;
  }
  baxter_core_msgs::EndpointState::ConstPtr getEndPoint()
  {
    return msg_;
  }
private:
  baxter_core_msgs::EndpointState::ConstPtr msg_;
};




void outputAsMatrix(const Eigen::Quaterniond& q, Eigen::Vector3d& r)
{
    Eigen::Matrix3d m;
    m=q.normalized().toRotationMatrix();
    r=m.eulerAngles(0, 1, 2);

}


//################CLASS#################################
int main(int argc, char **argv)
{
    //std::ofstream myfile;
  	//myfile.open ("my_example.txt");
    //ofstream cout("informacion_total.txt");
    std::ofstream outfile ;
    outfile.open("data_fullxd_motor.txt");
    //outfile << "Writing this to a file.\n";
    //############################################
    //INFORMACION DEL BAXTER
    //############################################
    std::string baxter_description = ros::package::getPath("baxter_description");
    std::string model_name = baxter_description + "/urdf/baxter.urdf";
    RobotModel* robot = new RobotModel();
    bool has_floating_base = false;
    if (!robot->loadURDF(model_name, has_floating_base)){
        return -1;
    }
    else{
        std::cout << "Robot " << model_name << " loaded." << std::endl;
    }
    unsigned int ndof_full = robot->ndof();
    std::cout << "Reading initial sensor values ..." << ndof_full<< std::endl;
    // Get the joint names and joint limits
    std::vector<std::string> jnames;
    std::vector<double> qmin, qmax, dqmax;
    jnames = robot->jointNames();
    qmin = robot->jointMinAngularLimits();
    qmax = robot->jointMaxAngularLimits();
    dqmax = robot->jointVelocityLimits();
    //############################################
    //NODE
    //############################################
    ros::init(argc, argv, "n_sendbaxter");
    ros::NodeHandle nh;
	Eigen::VectorXd P_right_wrist;
    Eigen::VectorXd P_left_elbow;
    Eigen::VectorXd P_right_elbow;
    Eigen::VectorXd P_left_wrist;

    MouseData mdata;
    EndPoint leftendpoint;
    EndPoint rightendpoint;

    for(int i=0;i<jnames.size();i++){
        cout<<jnames[i]<<endl;
    }
    //############################################
    //Suscriber
    //############################################
    ros::Subscriber sub_mouse = nh.subscribe("mouse", 1000 , &MouseData::readMouseData, &mdata);
    ros::Subscriber sub_endpoint_1 = nh.subscribe("robot/limb/right/endpoint_state", 1000, &EndPoint::readEndPoint, &rightendpoint);
    
    ros::Subscriber sub_endpoint_2 = nh.subscribe("robot/limb/left/endpoint_state", 1000, &EndPoint::readEndPoint, &leftendpoint);
    
    //ros::Subscriber sub_1 = nh.subscribe("tf", 1000, &TfPoints::readTfPoints, &tfpoints);
    JointSensors jsensor;
    ros::Subscriber sub_2 = nh.subscribe("robot/joint_states", 1000,&JointSensors::readJointSensors, &jsensor);
    //robot/joint_state ~ joint_state
    ros::Rate iter_rate(10000); // Hz
    unsigned int niter=0, max_iter = 1e3;
    unsigned int ndof_sensed = 222;
    ros::spinOnce();
    iter_rate.sleep();
    Eigen::VectorXd qsensed = Eigen::VectorXd::Zero(ndof_full);
    Eigen::Vector3d w(0,0,0);
    Eigen::Vector3d v(0,0,0);
    Eigen::Vector3d qwe(0,0,0);
    Eigen::Vector3d v_right(0,0,0);
    Eigen::Vector3d  v_left(0,0,0);

    Eigen::Quaterniond q_right;
    Eigen::Quaterniond q_left;
    Eigen::MatrixXd ball_position(3,100);
    Eigen::MatrixXd frame_pose(6,100);
    int link_number=
    {
        1
    };
    //Eigen::VectorXd qposition=Eigen::MatrixXd::Constant(15, 1, 0.0);
	std::vector< std::vector<double> > P_received;
    std::vector< std::vector<double> > P_send;
    std::vector< std::vector<double> > Q_orientation;
    //Q_orientation.resize(6);
    //############################################
    //INICIO DEL PROCESO DE ENVIO
    //############################################
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 500);
    ros::Publisher pub_left = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 500);
    ros::Publisher pub_right = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 500);
    
    sensor_msgs::JointState jcmd;
    baxter_core_msgs::JointCommand jcmd_left;
    baxter_core_msgs::JointCommand jcmd_right;

    jcmd_left.mode=1;
    jcmd_left.names.resize(7);
    jcmd_left.command.resize(7);

    jcmd_right.mode=1;
    jcmd_right.names.resize(7);
    jcmd_right.command.resize(7);
    
    std::cout<<"inicia vectores"<<std::endl;
    jcmd_left.names[0]="left_s0";
    jcmd_left.names[1]="left_s1";
    jcmd_left.names[2]="left_e0";
    jcmd_left.names[3]="left_e1";
    jcmd_left.names[4]="left_w0";
    jcmd_left.names[5]="left_w1";
    jcmd_left.names[6]="left_w2";

    jcmd_right.names[0]="right_s0";
    jcmd_right.names[1]="right_s1";
    jcmd_right.names[2]="right_e0";
    jcmd_right.names[3]="right_e1";
    jcmd_right.names[4]="right_w0";
    jcmd_right.names[5]="right_w1";
    jcmd_right.names[6]="rigth_w2";

    jcmd.name.resize(19);
    jcmd.position.resize(19);
    jcmd.velocity.resize(19);
    jcmd.effort.resize(19);


    jcmd.name[0]="head_pan";
    jcmd.name[1]="left_s0";
    jcmd.name[2]="left_s1";
    jcmd.name[3]="left_e0";
    jcmd.name[4]="left_e1";
    jcmd.name[5]="left_w0";
    jcmd.name[6]="left_w1";
    jcmd.name[7]="left_w2";


    jcmd.name[8]="right_s0";
    jcmd.name[9]="right_s1";
    jcmd.name[10]="right_e0";
    jcmd.name[11]="right_e1";
    jcmd.name[12]="right_w0";
    jcmd.name[13]="right_w1";
    jcmd.name[14]="right_w2";

    jcmd.name[15]="l_gripper_l_finger_joint";
    jcmd.name[16]="l_gripper_r_finger_joint";
    jcmd.name[17]="r_gripper_l_finger_joint";
    jcmd.name[18]="r_gripper_r_finger_joint";
    for(int j=0; j<19;j++){
        jcmd.position[j]=0.0;
    }

    pub.publish(jcmd);
    std::cout<<"jcmd ready"<<std::endl;
    pub_right.publish(jcmd_right);
    std::cout<<"jcmd_right ready"<<std::endl;
    pub_left.publish(jcmd_left);
    std::cout<<"jcmd_left ready"<<std::endl;
    //############################################
    // Tasks and Inverse Kinematics Solver
    //############################################
    unsigned int f = 1000;
    double dt = static_cast<double>(1.0/f);
    KineSolverWQP solver(robot, qsensed, dt);
    solver.setJointLimits(qmin, qmax, dqmax);
    KineTask* taskrh = new KineTaskPose(robot, 14, "position");
    taskrh->setGain(700.0);
    KineTask* tasklh = new KineTaskPose(robot, 7, "position");
    tasklh->setGain(700.0);
    KineTask* taskre = new KineTaskPose(robot, 11, "position");
    taskre->setGain(700.0);
    KineTask* taskle = new KineTaskPose(robot, 4, "position");
    taskle->setGain(700.0);
    Eigen::VectorXd qdes ;
    ros::Rate rate(f); // Hz
    //############################################
    //PROCESS MARKER
    //############################################
    double red[3] = {
        1.,0.,0.
    }
    ;
    double green[3] = {
        0.,1.,0.
    }
    ;
    double blue[3] = {
        0.,0.,1.
    }
    ;
    double yellow[3] = {
        0.5,0.5,0.
    }
    ;

    double white[3] = {
        1,1,1
    }
    ;

    BallMarker* b_shoulder_left;
    BallMarker* b_shoulder_right;
    BallMarker* b_elbow_right;
    BallMarker* b_elbow_left;
    BallMarker* b_hand_right;
    BallMarker* b_hand_left;

    b_shoulder_left= new BallMarker(nh, white);
    b_shoulder_right= new BallMarker(nh, white);
    b_elbow_right= new BallMarker(nh, yellow);
   	b_elbow_left= new BallMarker(nh, green);
    b_hand_right= new BallMarker(nh, red);
    b_hand_left= new BallMarker(nh, blue);

    solver.pushTask(taskrh);
  	solver.pushTask(tasklh);
  	solver.pushTask(taskre);
  	solver.pushTask(taskle);

    //############################################
    //ROS
    //############################################
    P_received.resize(6);
    P_send.resize(6);

    tf::StampedTransform transform_right_elbow;
    tf::StampedTransform transform_left_elbow;
    tf::StampedTransform transform_right_shoulder;
    tf::StampedTransform transform_left_shoulder;
    tf::StampedTransform transform_right_hand;
    tf::StampedTransform transform_left_hand;
    tf::TransformListener listener;
    double k=1;
    double hand_position=0;
    double hola=0;
    while(ros::ok())
    {
        if(rightendpoint.getEndPoint()->pose.position.x!=0){
        //outfile  <<rightendpoint.getEndPoint()->pose.position.x<<" ";
        //outfile  <<rightendpoint.getEndPoint()->pose.position.y<<" ";
        //outfile  <<rightendpoint.getEndPoint()->pose.position.z<<" ";
        
        //outfile  <<leftendpoint.getEndPoint()->pose.position.x<<" ";
        //outfile  <<leftendpoint.getEndPoint()->pose.position.y<<" ";
        //outfile  <<leftendpoint.getEndPoint()->pose.position.z<<" ";
 	    }

        
        int mouse_es=mdata.getData()->y;
        hand_position=mdata.getData()->x;
        jcmd.header.stamp = ros::Time::now();
 
        //############################################
        //BAXTER INFO
        //############################################
        double L1 = 0.46;//Del hombro al codo
        double L2 = 0.50;//Del codo a la mano

        double shoulder_x=-0.10;
        double shoulder_y=-0.34;
        double shoulder_z=0.41;

        for (int m=0;m<6;m++){
				P_received[m].resize(7);
				P_send[m].resize(3);
		}
		std::cout<<"bien"<<std::endl;
		//usleep(1000);
        //############################################
        try
		    {
		    	listener.lookupTransform("/openni_depth_frame", "/right_elbow_1",ros::Time(0), transform_right_elbow);
		    	listener.lookupTransform("/openni_depth_frame", "/left_elbow_1",ros::Time(0), transform_left_elbow);
		    	listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1",ros::Time(0), transform_right_shoulder);
		    	listener.lookupTransform("/openni_depth_frame", "/left_shoulder_1",ros::Time(0), transform_left_shoulder);
		    	listener.lookupTransform("/openni_depth_frame", "/right_hand_1",ros::Time(0), transform_right_hand);
		    	listener.lookupTransform("/openni_depth_frame", "/left_hand_1",ros::Time(0), transform_left_hand);
		    }

		    catch (tf::TransformException ex)
		    {
		      	ROS_ERROR("%s",ex.what());
		      	ros::Duration(2.0).sleep();
		    }


        if (transform_right_shoulder.getOrigin().x()>0.25)
        {
	       

                /*
             outfile  <<mdata.getData()->x<<" ";
            outfile  <<mdata.getData()->y<<" ";
            outfile  <<mdata.getData()->z<<" ";

            std::cout<<"recibe"<<std::endl;
	        std::cout<<"right_shoulder_x: "<<transform_right_shoulder.getOrigin().x()<<std::endl;
			std::cout<<"right_shoulder_y: "<<transform_right_shoulder.getOrigin().y()<<std::endl;
		    std::cout<<"right_shoulder_z: "<<transform_right_shoulder.getOrigin().z()<<std::endl;
		    std::cout<<"left_shoulder_x: "<<transform_left_shoulder.getOrigin().x()<<std::endl;
			std::cout<<"left_shoulder_y: "<<transform_left_shoulder.getOrigin().y()<<std::endl;
		    std::cout<<"left_shoulder_z: "<<transform_left_shoulder.getOrigin().z()<<std::endl;

		    std::cout<<"right_elbow_x: "<<transform_right_elbow.getOrigin().x()<<std::endl;
			std::cout<<"right_elbow_y: "<<transform_right_elbow.getOrigin().y()<<std::endl;
		    std::cout<<"right_elbow_z: "<<transform_right_elbow.getOrigin().z()<<std::endl;
		    std::cout<<"left_elbow_x: "<<transform_left_elbow.getOrigin().x()<<std::endl;
			std::cout<<"left_elbow_y: "<<transform_left_elbow.getOrigin().y()<<std::endl;
		    std::cout<<"left_elbow_z: "<<transform_left_elbow.getOrigin().z()<<std::endl;
             */
		    //outfile  << transform_right_hand.getOrigin().x()<< " "<<transform_right_hand.getOrigin().y()<<" "<<transform_right_hand.getOrigin().z()<< " ";
            std::cout<<"right_hand_x: "<<transform_right_hand.getOrigin().x()<<std::endl;
            std::cout<<"right_hand_y: "<<transform_right_hand.getOrigin().y()<<std::endl;
		    std::cout<<"right_hand_z: "<<transform_right_hand.getOrigin().z()<<std::endl;
		    //outfile  <<transform_left_hand.getOrigin().x() << " "<< transform_left_hand.getOrigin().y() << " "<<transform_left_hand.getOrigin().z()<<".\n";
            std::cout<<"left_hand_x: "<<transform_left_hand.getOrigin().x()<<std::endl;
			std::cout<<"left_hand_y: "<<transform_left_hand.getOrigin().y()<<std::endl;
		    std::cout<<"left_hand_z: "<<transform_left_hand.getOrigin().z()<<std::endl;

           
		    P_received.at(0).at(0)=transform_right_shoulder.getOrigin().x();
			P_received.at(0).at(1)=transform_right_shoulder.getOrigin().y();
		    P_received.at(0).at(2)=transform_right_shoulder.getOrigin().z();
		    
		    P_received.at(3).at(0)=transform_left_shoulder.getOrigin().x();
			P_received.at(3).at(1)=transform_left_shoulder.getOrigin().y();
		    P_received.at(3).at(2)=transform_left_shoulder.getOrigin().z();

		    P_received.at(1).at(0)=transform_right_elbow.getOrigin().x();
			P_received.at(1).at(1)=transform_right_elbow.getOrigin().y();
		    P_received.at(1).at(2)=transform_right_elbow.getOrigin().z();
		    
		    P_received.at(4).at(0)=transform_left_elbow.getOrigin().x();
			P_received.at(4).at(1)=transform_left_elbow.getOrigin().y();
		    P_received.at(4).at(2)=transform_left_elbow.getOrigin().z();

            
		    P_received.at(2).at(0)=transform_right_hand.getOrigin().x();
            P_received.at(2).at(1)=transform_right_hand.getOrigin().y();
            P_received.at(2).at(2)=transform_right_hand.getOrigin().z();
            q_right.x()=transform_right_hand.getRotation().x();
		    q_right.y()=transform_right_hand.getRotation().y();
            q_right.z()=transform_right_hand.getRotation().z();
            q_right.w()=transform_right_hand.getRotation().w();
            outputAsMatrix(q_right, v_right);
            //std::cout<<"rigt: "<<v_right<<std::endl;
            
		    P_received.at(5).at(0)=transform_left_hand.getOrigin().x();
			P_received.at(5).at(1)=transform_left_hand.getOrigin().y();
		    P_received.at(5).at(2)=transform_left_hand.getOrigin().z();
            q_left.x()=transform_left_hand.getRotation().x();
            q_left.y()=transform_left_hand.getRotation().y();
            q_left.z()=transform_left_hand.getRotation().z();
            q_left.w()=transform_left_hand.getRotation().w();
            outputAsMatrix(q_left, v_left);
            //std::cout<<"left: "<<v_left<<std::endl;

		    double M1= sqrt(pow(P_received[1][0]- P_received[0][0], 2.0) + pow(P_received[1][1]- P_received[0][1], 2.0) + pow(P_received[1][2]- P_received[0][2], 2.0));
            double M2= sqrt(pow(P_received[2][0]- P_received[1][0], 2.0) + pow(P_received[2][1]- P_received[1][1], 2.0) + pow(P_received[2][2]- P_received[1][2], 2.0));
            double Q1 = L1 / M1;
            double Q2 = L2 / M2;

            P_send.at(0).at(0) = shoulder_x;
            P_send.at(0).at(1) = shoulder_y;
            P_send.at(0).at(2) = shoulder_z;

            //Redefinimos P2
            P_send.at(1).at(0) = Q1*(P_received.at(1).at(0) - P_received.at(0).at(0))+P_send.at(0).at(0);
            P_send.at(1).at(1) = Q1*(P_received.at(1).at(1) - P_received.at(0).at(1))+P_send.at(0).at(1);
            P_send.at(1).at(2) = Q1*(P_received.at(1).at(2) - P_received.at(0).at(2))+P_send.at(0).at(2);

            //Redefinimos P2
            P_send.at(2).at(0) = Q2*(P_received.at(2).at(0) - P_received.at(1).at(0))+P_send.at(1).at(0);
            P_send.at(2).at(1) = Q2*(P_received.at(2).at(1) - P_received.at(1).at(1))+P_send.at(1).at(1);
            P_send.at(2).at(2) = Q2*(P_received.at(2).at(2) - P_received.at(1).at(2))+P_send.at(1).at(2);
            //Redfinimos P1

		    double M3= sqrt(pow(P_received.at(4).at(0)- P_received.at(3).at(0), 2.0) + pow(P_received.at(4).at(1)- P_received.at(3).at(1), 2.0) + pow(P_received.at(4).at(2)- P_received.at(3).at(2), 2.0));
            double M4= sqrt(pow(P_received.at(5).at(0)- P_received.at(4).at(0), 2.0) + pow(P_received.at(5).at(1)- P_received.at(4).at(1), 2.0) + pow(P_received.at(5).at(2)- P_received.at(4).at(2), 2.0));
            double Q3 = L1 / M3;
            double Q4 = L2 / M4;

            P_send[3][0] = shoulder_x;
            P_send[3][1] = -shoulder_y;
            P_send[3][2] = shoulder_z;

            //Redefinimos P2
            P_send[4][0] = Q3*(P_received[4][0] - P_received[3][0])+P_send[3][0];
            P_send[4][1] = Q3*(P_received[4][1] - P_received[3][1])+P_send[3][1];
            P_send[4][2] = Q3*(P_received[4][2] - P_received[3][2])+P_send[3][2];

            //Redefinimos P2
            P_send[5][0] = Q4*(P_received[5][0] - P_received[4][0])+P_send[4][0];
            P_send[5][1] = Q4*(P_received[5][1] - P_received[4][1])+P_send[4][1];
            P_send[5][2] = Q4*(P_received[5][2] - P_received[4][2])+P_send[4][2];

//######################### Rotation ############################
            P_send[0][0]=-P_send[0][0];
            P_send[0][1]=-P_send[0][1];
            P_send[0][2]=+P_send[0][2];

            P_send[1][0]=-P_send[1][0];
            P_send[1][1]=-P_send[1][1];
            P_send[1][2]=+P_send[1][2];

            P_send[2][0]=-P_send[2][0];
            P_send[2][1]=-P_send[2][1];
            P_send[2][2]=+P_send[2][2];

            P_send[3][0]=-P_send[3][0];
            P_send[3][1]=-P_send[3][1];
            P_send[3][2]=+P_send[3][2];

            P_send[4][0]=-P_send[4][0];
            P_send[4][1]=-P_send[4][1];
            P_send[4][2]=+P_send[4][2];

            P_send[5][0]=-P_send[5][0];
            P_send[5][1]=-P_send[5][1];
            P_send[5][2]=+P_send[5][2];

            std::cout<<"Valores Obtenidos"<<std::endl;

//###################################################################
//######################### ##### ############################
		    P_right_wrist.resize(6);
		    //std::cout<<"1"<<std::endl;
		    P_left_wrist.resize(6);
		    //std::cout<<"2"<<std::endl;
		    P_right_elbow.resize(3);
		    //std::cout<<"3"<<std::endl;
		    P_left_elbow.resize(3);
		    //std::cout<<"4"<<std::endl;
		    //std::cout<<"Vectores Inicializados"<<std::endl;
            // Red
            P_right_wrist[0]=P_send[5][0];
            P_right_wrist[1]=P_send[5][1];
            P_right_wrist[2]=P_send[5][2];

            //Blue
            P_left_wrist[0]=P_send[2][0];
            P_left_wrist[1]=P_send[2][1];
            P_left_wrist[2]=P_send[2][2];

            //yellow
            P_right_elbow[0]=P_send[4][0];
            P_right_elbow[1]=P_send[4][1];
            P_right_elbow[2]=P_send[4][2];
            //Green
            P_left_elbow[0]=P_send[1][0];
            P_left_elbow[1]=P_send[1][1];
            P_left_elbow[2]=P_send[1][2];

            //std::cout<<"task"<<std::endl;
            //std::cout<<"P_left_elbow"<<std::endl<<P_left_elbow<<std::endl;
            taskle->setDesiredValue(P_left_elbow);
            //std::cout<<"P_left_wrist"<<std::endl<<P_left_wrist<<std::endl;
            tasklh->setDesiredValue(P_left_wrist);
            //std::cout<<"P_right_elbow"<<std::endl<<P_right_elbow<<std::endl;
            taskre->setDesiredValue(P_right_elbow);
            //std::cout<<"P_right_wrist"<<std::endl<<P_right_wrist<<std::endl;
            taskrh->setDesiredValue(P_right_wrist);

 			if(1<2){
            //std::cout<<"solver"<<std::endl;
            solver.getPositionControl(qsensed, qdes);
           	
            for(int j=0; j<jnames.size();j++)
            {
               jcmd.position[j]=qdes[j];
            }

            for(int i=0; i<7;i++)
            {
               jcmd_left.command[i]=qdes[i+1];
            }

            for(int l=0; l<7;l++)
            {
               jcmd_right.command[l]=qdes[l+8];
            }
            
            //outfile  << jcmd_left.command[0]<<" ";
            //outfile  << jcmd_left.command[1]<<" ";
            //outfile  << jcmd_left.command[3]<<".\n";


           
            if(mouse_es==1){
                if(hand_position>3.00 || hand_position<-3.00  ){
                k=k*-1;
                }
                hola=hand_position+0.3*k;
            }
            
            //std::cout<<"hola: "<<hola<<std::endl;
            //jcmd_right.command[6]=hola;
            //std::cout<<"publish"<<std::endl;
            
            pub.publish(jcmd);
            pub_right.publish(jcmd_right);
            pub_left.publish(jcmd_left);
            
            }
            qsensed = qdes;
//#################################################################################
            ball_position(0,0)=P_send[0][0];
            ball_position(1,0)=P_send[0][1];
            ball_position(2,0)=P_send[0][2];

            ball_position(0,1)=P_send[1][0];
            ball_position(1,1)=P_send[1][1];
            ball_position(2,1)=P_send[1][2];
            
            ball_position(0,2)=P_send[2][0];
            ball_position(1,2)=P_send[2][1];
            ball_position(2,2)=P_send[2][2];
            
            ball_position(0,3)=P_send[3][0];
            ball_position(1,3)=P_send[3][1];
            ball_position(2,3)=P_send[3][2];
            
            ball_position(0,4)=P_send[4][0];
            ball_position(1,4)=P_send[4][1];
            ball_position(2,4)=P_send[4][2];
            
            ball_position(0,5)=P_send[5][0];
            ball_position(1,5)=P_send[5][1];
            ball_position(2,5)=P_send[5][2];

            b_shoulder_left->setPose(ball_position.col(0));
	        b_shoulder_right->setPose(ball_position.col(3));
	        b_elbow_right->setPose(ball_position.col(4));
	        b_elbow_left->setPose(ball_position.col(1));
	        b_hand_right->setPose(ball_position.col(5));
	        b_hand_left->setPose(ball_position.col(2));

	        b_shoulder_left->publish();
	        b_shoulder_right->publish();
	        b_elbow_right->publish();
	        b_elbow_left->publish();
	        b_hand_right->publish();
	        b_hand_left->publish();

        }

        ros::spinOnce();
        rate.sleep();
    }
    outfile.close();
    return 0;
}




/*if (kpoints.getPoints()->pose.size() > 0){
//Programacion que recibe el brazo derecho
//for (int k=0;k<3;k++){
//P[k].resize(3);
//It's necessary reduce P[0][k]=(0,0,0)
//P[k][0] = (-kpoints.getPoints()->pose[k].position.z)-(-kpoints.getPoints()->pose[0].position.z);
//P[k][1] = (-kpoints.getPoints()->pose[k].position.x)-(-kpoints.getPoints()->pose[0].position.x);
//P[k][2] = (kpoints.getPoints()->pose[k].position.y)-(kpoints.getPoints()->pose[0].position.y);
//}
//Construimos los puntos (0,0,0); P1; P2
//Hallamos el modulo M1 de P1 - modulo M2 de (P2-P1)
//double M1= sqrt(pow(P[1][0],2.0)+pow(P[1][1],2.0)+pow(P[1][2],2.0));
//double M2= sqrt(pow(P[2][0]- P[1][0], 2.0) + pow(P[2][1]- P[1][1], 2.0) + pow(P[2][2]- P[1][2], 2.0));
//double Q1 = L1 / M1;
            double Q2 = L2 / M2;
            //Redfinimos P1
            P[0][0] = elbow_x;
            P[0][1] = elbow_y;
            P[0][2] = elbow_z;
            //Redefinimos P2
            P[2][0] = Q2*(P[2][0] - P[1][0]);
            P[2][1] = Q2*(P[2][1] - P[1][1]);
            P[2][2] = Q2*(P[2][2] - P[1][2]);
            //Redfinimos P1
            P[1][0] = P[0][0]+Q1*P[1][0];
            P[1][1] = P[0][1]+Q1*P[1][1];
            P[1][2] = P[0][2]+Q1*P[1][2];
            
            P[2][0] = P[2][0]+P[1][0];
            P[2][1] = P[2][1]+P[1][1];
            P[2][2] = P[2][2]+P[1][2];
            //Programacion que recibe el brazo izquierdo
            for (unsigned k=(P.size()/2);k<P.size();k++){
                P[k].resize(3);
                P[k][0] = (-kpoints.getPoints()->pose[k].position.z)-(-kpoints.getPoints()->pose[3].position.z);
                P[k][1] = (-kpoints.getPoints()->pose[k].position.x)-(-kpoints.getPoints()->pose[3].position.x);
                P[k][2] = (kpoints.getPoints()->pose[k].position.y)-(kpoints.getPoints()->pose[3].position.y);
            }
            //Hallamos el modulo M1 de P4
            double M3 = sqrt(pow(P[4][0], 2.0) + pow(P[4][1], 2.0) + pow(P[4][2], 2.0));
            double M4 = sqrt(pow(P[5][0] - P[4][0], 2.0) + pow(P[5][1] - P[4][1], 2.0) + pow(P[5][2] - P[4][2], 2.0));
            double Q3 = L1 / M3;
            double Q4 = L2 / M4;
            //Construimos los puntos (0,0,0); P4; P5
            P[3][0] = elbow_x;
            P[3][1] = -elbow_y;
            P[3][2] = elbow_z;
            //Redefinimos P5
            P[5][0] = Q4*(P[5][0] - P[4][0]);
            P[5][1] = Q4*(P[5][1] - P[4][1]);
            P[5][2] = Q4*(P[5][2] - P[4][2]);
            //Redfinimos P4
            P[4][0] = P[3][0]+Q3*P[4][0];
            P[4][1] = P[3][1]+Q3*P[4][1];
            P[4][2] = P[3][2]+Q3*P[4][2];
            
            P[5][0] = P[5][0]+P[4][0];
            P[5][1] = P[5][1]+P[4][1];
            P[5][2] = P[5][2]+P[4][2];
            
            //cout<<"error1"<<endl;
            //############################################
        	//trasform Orientation
        	//############################################

            for (int k=0;k<6;k++){
                Q_orientation[k].resize(3);
                tf::Quaternion q(kpoints.getPoints()->pose[k].orientation.x, kpoints.getPoints()->pose[k].orientation.y, kpoints.getPoints()->pose[k].orientation.z, kpoints.getPoints()->pose[k].orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                Q_orientation[k][0]=roll;
                Q_orientation[k][1]=pitch;
                Q_orientation[k][2]=yaw;
            }
            //outfile.put(Q_orientation[1][0]);
            P_right_wrist.resize(3);
            P_left_wrist.resize(3);
            P_right_elbow.resize(3);
            P_left_elbow.resize(3);
            
            /*
            //Elbow Izquierdo
            P_left_elbow[3] = Q_orientation[1][0];
            P_left_elbow[4] = Q_orientation[1][1];
            P_left_elbow[5] = Q_orientation[1][2];
            //Left hand
            P_left_wrist[3] = Q_orientation[2][0];
            P_left_wrist[4] = Q_orientation[2][1];
            P_left_wrist[5] = Q_orientation[2][2];
            //Right elbow
            P_right_elbow[3] = Q_orientation[4][0];
            P_right_elbow[4] = Q_orientation[4][1];
            P_right_elbow[5] = Q_orientation[4][2];
            //Right hand
            double x = 0.635163;
            double y= -0.830166;
            double z=0.320976;****
            pub.publish(jcmd);
            P_right_wrist[0] = P[5][0];//0.635163;//
            P_right_wrist[1] = P[5][1];//-0.830166;//
            P_right_wrist[2] = P[5][2];//0.320976;//
            
            valor=valor-0.0001;
            //cout<<valor<<endl;
            //P_right_wrist[3] = Q_orientation[5][0];//1.5708;//
            //P_right_wrist[4] = Q_orientation[5][1];//-0.7854;//
            //P_right_wrist[5] = Q_orientation[5][2];//0;//
            //cout<<"error2"<<endl;

            // Format file: Position-Orientation:	
            for(int q=0;q<6;q++){
            	for(int r=0;r<3;r++){
            		myfile << P[q][r]<<endl;
            	}
            	for(int r=0;r<3;r++){
            		myfile << Q_orientation[q][r]<<endl;
            	}
            }
           

            P_right_elbow[0] = P[4][0];
            P_right_elbow[1] = P[4][1];
            P_right_elbow[2] = P[4][2];
            
            P_left_wrist[0] = P[2][0];
            P_left_wrist[1] = P[2][1];
            P_left_wrist[2] = P[2][2];
            
            P_left_elbow[0] = P[1][0];
            P_left_elbow[1] = P[1][1];
            P_left_elbow[2] = P[1][2];
             /*
            P_right_wrist[3] = Q_orientation[5][0];
            P_right_wrist[4] = Q_orientation[5][1];
            P_right_wrist[5] = Q_orientation[5][2];
            ***
            taskle->setDesiredValue(P_left_elbow);
            //cout<<"P_left_elbow"<<endl;
            //cout<<P_left_elbow.transpose()<<endl;
            tasklh->setDesiredValue(P_left_wrist);
            //cout<<"P_left_wrist"<<endl;
            //cout<<P_left_wrist.transpose()<<endl;
            taskre->setDesiredValue(P_right_elbow);
            //cout<<"P_right_elbow"<<endl;
            //cout<<P_right_elbow.transpose()<<endl;
            taskrh->setDesiredValue(P_right_wrist);
            //cout<<"P_right_wrist"<<endl;
            //cout<<P_right_wrist.transpose()<<endl;
            //cout<<"KineSolver"<<endl;
            solver.getPositionControl(qsensed, qdes);
            //cout<<"publish"<<endl;
            //cout<<"qdes:_"<<endl<<qdes<<endl;
            
            for(int j=0; j<jnames.size();j++){
               jcmd.position[j]=qdes[j];
            }
            pub.publish(jcmd);
            //cout<<"publish"<<endl;
            qsensed = qdes;
            ball_position(0,0)=P[3][0];
            ball_position(1,0)=P[3][1];
            ball_position(2,0)=P[3][2];
            ball_position(0,1)=P[4][0];
            ball_position(1,1)=P[4][1];
            ball_position(2,1)=P[4][2];
            ball_position(0,2)=P[5][0];
            ball_position(1,2)=P[5][1];
            ball_position(2,2)=P[5][2];
            ball_position(0,3)=P[0][0];
            ball_position(1,3)=P[0][1];
            ball_position(2,3)=P[0][2];
            ball_position(0,4)=P[1][0];
            ball_position(1,4)=P[1][1];
            ball_position(2,4)=P[1][2];
            ball_position(0,5)=P[2][0];
            ball_position(1,5)=P[2][1];
            ball_position(2,5)=P[2][2];

            for (int k=0; k<6;k++){
	            for(int j=0; j<3;j++){
	            	frame_pose(j,k)=ball_position(j,k);
	            }
        	}

        	for (int k=0; k<6;k++){
	            for(int j=3; j<6;j++){
	            	frame_pose(j,k)=Q_orientation[k][j-3];
	            }
        	}
            marker_current_right_0->setPose(ball_position.col(0));
	        marker_current_right_1->setPose(ball_position.col(1));
	        marker_current_right_2->setPose(ball_position.col(2));
	        marker_current_left_0->setPose(ball_position.col(3));
	        marker_current_left_1->setPose(ball_position.col(4));
	        marker_current_left_2->setPose(ball_position.col(5));

	        frame_current_right_0->setPose(frame_pose.col(0));
    		frame_current_right_1->setPose(frame_pose.col(1));
    		frame_current_right_2->setPose(frame_pose.col(2));
    		frame_current_left_0->setPose(frame_pose.col(3));
    		frame_current_left_1->setPose(frame_pose.col(4));
    		frame_current_left_2->setPose(frame_pose.col(5));

	        marker_current_right_0->publish();
	        marker_current_right_1->publish();
	        marker_current_right_2->publish();
	        marker_current_left_0->publish();
	        marker_current_left_1->publish();
	        marker_current_left_2->publish();

	        frame_current_right_0->publish();
    		frame_current_right_1->publish();
    		frame_current_right_2->publish();
    		frame_current_left_0->publish();
    		frame_current_left_1->publish();
    		frame_current_left_2->publish();
        }
        */