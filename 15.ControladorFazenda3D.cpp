#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <math.h>
#include <tf/tf.h>
#include "libs/kbhit.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include "FLIE-master/flie.h"
#define DOS
using namespace std;

//----Fuzzy-----//
fuzzy_control fcDesvLin, fcDesvAng, fcNavLin, fcNavAng;

//----Variáveis Globais-----//
nav_msgs::Odometry feedback;
nav_msgs::Odometry feedback2;
sensor_msgs::LaserScan sensor;
sensor_msgs::LaserScan sensor2;
geometry_msgs::Twist msg, msg2;
std_msgs::Float32 flagArm,flagArm2;
const float PI = 3.14;
float tolerance_orie = 0.001, tolerance_pos = 0.0175;
float posdesejada[2], oridesejada;
float posdesejada2[2], oridesejada2;
float min_dist = 99, erroLin = 99, erroAng = 99;
float min_dist2 = 99, erroLin2 = 99, erroAng2 = 99;
float prox_desejada = 0, ang_desejado = 0;
float prox_desejada2 = 0, ang_desejado2 = 0;
int maquina1, maquina2;
vector<int> ordem;
double yaw_angle = 0, yaw_angle2 = 0;
char a=0;
enum robos {robo1,robo2};

//----Maquina de estados-----//
enum estado { definindo, indo, arrumando, aproximando, garra, afastando};
int estadoAtual = definindo, estadoAtual2 = definindo;
int flags=1,flags2=1;
int flagG=0,flagG2=0;

//---Declaração de Funções---//
float controlePos(float posdesejada[2], float oridesejada, nav_msgs::Odometry feedback, ros::Publisher publicador, sensor_msgs::LaserScan sensor, float angule, int rob);
float calcErrLin(float posdesejada[2], nav_msgs::Odometry feedback);
float calcErrAng(float posdesejada[2], nav_msgs::Odometry feedback, float yaw_angle);
void executa_mde(int robo, ros::Publisher pub, ros::Publisher pubArm);
void movFixRobo(float vel,geometry_msgs::Twist msg, ros::Publisher publicador);
void subCallback(const nav_msgs::Odometry::ConstPtr& msg);
void subCallback2(const nav_msgs::Odometry::ConstPtr& msg2);
void subSense(const sensor_msgs::LaserScan::ConstPtr& sns);
void subSense2(const sensor_msgs::LaserScan::ConstPtr& sns2);
void subGarra(const std_msgs::Float32 gr);
void subGarra2(const std_msgs::Float32 gr2);
void setMaq(int maq, int robo, float posdesejada[2]);
void startFcDesvLin();
void startFcDesvAng();
void startFcNavLin();
void startFcNavAng();

//---Funções---//
int main(int argc, char **argv)
{
	//----ROS config-------//
	ros::init(argc, argv, "ROB_controle_posicao");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Publisher pub2 = n.advertise<geometry_msgs::Twist>("cmd_vel2", 1000);
	ros::Publisher pub3 = n.advertise<std_msgs::Float32>("cmd_arm", 1000);
	ros::Publisher pub4 = n.advertise<std_msgs::Float32>("cmd_arm2", 1000);
	ros::Subscriber sub = n.subscribe("odomR1", 1000, subCallback);
	ros::Subscriber sub2 = n.subscribe("odomR2", 1000, subCallback2);
	ros::Subscriber subG1 = n.subscribe("garra1", 1000, subGarra);
	ros::Subscriber subG2 = n.subscribe("garra2", 1000, subGarra2);
	ros::Subscriber subSens = n.subscribe("scan", 1000, subSense);
	ros::Subscriber subSens2 = n.subscribe("scan2", 1000, subSense2);
	ros::Rate loop_rate(10);

	if (ros::ok())
	{
		startFcDesvLin();
		startFcDesvAng();
		startFcNavLin();
		startFcNavAng();

		cout << "Digite a ordem das peças a serem recolhidas\n";
		for (int i=0;i<=5;i++)
		{
			printf("%iº>>",i+1);
			cin >> maquina1;
			ordem.push_back(maquina1);
		}
		
		setMaq(ordem[0],1,posdesejada);
		ordem.erase(ordem.begin());
		setMaq(ordem[0],2,posdesejada2);
		ordem.erase(ordem.begin());

		ros::spinOnce();
		loop_rate.sleep();

		while ((flags <= 4) || (flags2 <= 4))
		{
			system("clear");
			ROS_INFO("ROBO1\n Estado:%i----------------------\nflag: %i\ngarra: %i",estadoAtual,flags,flagG);
			executa_mde(robo1,pub1,pub3);
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("ROBO2\n Estado:%i----------------------\nflag: %i\ngarra: %i",estadoAtual2,flags2,flagG2);
			executa_mde(robo2,pub2,pub4);
			ros::spinOnce();
			loop_rate.sleep();
			if ( kbhit() )
			{
				a = getchar();
				if(a==113) 
				{
					flags = 5; 
					flags2 = 5;
				}
				ROS_INFO("Pressionado: %c",a);
			}
		}
		
		msg.linear.x = 0;
		msg.angular.z = 0;
		pub1.publish(msg);
		pub2.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		ROS_WARN("...TUDO PRONTO...");
	}
	return 0;
}

void executa_mde(int robo, ros::Publisher pub, ros::Publisher pubArm)
{	
	float min_dist = 99,min_dist2 = 99;
	switch(robo)
	{
		case robo1:
			switch(estadoAtual)
			{
				case definindo:
					switch(flags)
					{
						case 1:
							erroLin = calcErrLin(posdesejada,feedback);
							prox_desejada = 0.1;
							ang_desejado=PI/2;
							estadoAtual=indo;
						break;
						case 2:
							posdesejada[0]=-0.6;
							posdesejada[1]=0.8;
							prox_desejada=0.2;
							ang_desejado=-PI/2;
							erroLin = calcErrLin(posdesejada,feedback);
							estadoAtual=indo;
						break;
						case 3:
							posdesejada[0]=0;
							posdesejada[1]=0.5;
							prox_desejada=0.4;
							ang_desejado=PI/2;
							erroLin = calcErrLin(posdesejada,feedback);
							estadoAtual=indo;
						break;
						case 4:
							movFixRobo(0,msg,pub);
							if(ordem.empty()) flags++;
							else
							{
								flags=1;
								setMaq(ordem[0],1,posdesejada);
								ordem.erase(ordem.begin());
							}
						break;
						case 5:
							movFixRobo(0,msg,pub);
						break;
					}
				break;
				case indo:
					if (erroLin >= tolerance_pos)
					{
						erroLin=controlePos(posdesejada, 0, feedback, pub, sensor, yaw_angle, 1);
					}
					else
					{
						movFixRobo(0,msg,pub);
						erroAng = calcErrAng(posdesejada,feedback,yaw_angle);
						estadoAtual=arrumando;
					}
				break;
				case arrumando:
					if(abs(erroAng) >= tolerance_orie)
					{
						erroAng=controlePos(posdesejada, ang_desejado, feedback, pub, sensor, yaw_angle, 1);
					}
					else
					{
						if (flags==3)
						{
							flags=flags+1;
							estadoAtual=definindo;
						}
						else 
						{
							estadoAtual=aproximando;
							flagArm.data=1;
							pubArm.publish(flagArm);
						}
					}
				break;
				case aproximando:
					min_dist = *min_element(sensor.ranges.begin(),sensor.ranges.end());
					if (min_dist >= prox_desejada)
					{
						movFixRobo(0.15,msg,pub);
					}
					else
					{
						estadoAtual=garra;
					}					
				break;
				case garra: 
						movFixRobo(0,msg,pub);
			
						switch (flags)
						{
							case 1: flagArm.data=4; break;
							case 2: flagArm.data=3; pubArm.publish(flagArm);break;
							case 3: flagArm.data=4; break;
						}
						
						if (flagG==1) 
						{
							pubArm.publish(flagArm);
							estadoAtual=afastando;
						}
				break;
				case afastando: 
					flagArm.data=2;
					pubArm.publish(flagArm);
					min_dist = *min_element(sensor.ranges.begin(),sensor.ranges.end());
					if (min_dist <= 0.5)
					{
						movFixRobo(-0.3,msg,pub);
					}
					else
					{
						movFixRobo(0,msg,pub);
						flags=flags+1;
						estadoAtual=definindo;
					}
				break;
			}
		break;
		case robo2:
			switch(estadoAtual2)
			{
				case definindo:
					switch(flags2)
					{
						case 1:
							erroLin2 = calcErrLin(posdesejada2,feedback2);
							prox_desejada2 = 0.1;
							ang_desejado2 = PI/2;
							estadoAtual2=indo;
						break;
						case 2:
							posdesejada2[0]=0.6;
							posdesejada2[1]=0.8;
							prox_desejada2=0.2;
							ang_desejado2=-PI/2;
							erroLin2 = calcErrLin(posdesejada2,feedback2);
							estadoAtual2=indo;
						break;
						case 3:
							posdesejada2[0]=0;
							posdesejada2[1]=0.5;
							prox_desejada2=0.4;
							ang_desejado2=PI/2;
							erroLin2 = calcErrLin(posdesejada2,feedback2);
							estadoAtual2=indo;
						break;
						case 4:
							movFixRobo(0,msg,pub);
							if(ordem.empty()) flags2++;
							else
							{
								flags2=1;
								setMaq(ordem[0],2,posdesejada2);
								ordem.erase(ordem.begin());
							}
						break;
						case 5:
							movFixRobo(0,msg,pub);
						break;
					}
				break;
				case indo:
					if (erroLin2 >= tolerance_pos)
					{
						erroLin2=controlePos(posdesejada2, 0, feedback2, pub, sensor2, yaw_angle2, 2);
					}
					else
					{
						movFixRobo(0,msg,pub);
						erroAng2 = calcErrAng(posdesejada2,feedback2,yaw_angle2);
						estadoAtual2=arrumando;
					}
				break;
				case arrumando:
					if(abs(erroAng2) >= tolerance_orie)
					{
						erroAng2=controlePos(posdesejada2, ang_desejado2, feedback2, pub, sensor2, yaw_angle2, 2);
					}
					else
					{
						if (flags2==3)
						{
							flags2=flags2+1;
							estadoAtual2=definindo;
						}
						else 
						{						
							flagArm2.data=1;
							pubArm.publish(flagArm2);
							estadoAtual2=aproximando;
						}
					}
				break;
				case aproximando:
					min_dist2 = *min_element(sensor2.ranges.begin(),sensor2.ranges.end());
					if (min_dist2 >= prox_desejada2)
					{
						movFixRobo(0.15,msg,pub);
					}
					else
					{
						estadoAtual2=garra;
					}					
				break;
				case garra: 
						movFixRobo(0,msg,pub);
						switch (flags2)
						{
							case 1: flagArm2.data=4; break;
							case 2: flagArm2.data=3; pubArm.publish(flagArm2);break;
							case 3: flagArm2.data=4; break;
						}
						
						if (flagG2==1) 
						{
							pubArm.publish(flagArm2);
							estadoAtual2=afastando;
						}
				break;
				case afastando: 
					flagArm2.data=2;
					pubArm.publish(flagArm2);
					min_dist2 = *min_element(sensor2.ranges.begin(),sensor2.ranges.end());
					if (min_dist2 <= 0.5)
					{
						movFixRobo(-0.3,msg,pub);
					}
					else
					{
						movFixRobo(0,msg,pub);
						flags2=flags2+1;
						estadoAtual2=definindo;
					}
				break;
			}
		break;
	}
}
float calcErrLin(float pos[2],nav_msgs::Odometry retorno)
{
	return sqrt(pow(pos[0]-retorno.pose.pose.position.x,2)+pow(pos[1]-retorno.pose.pose.position.y,2));
}

float calcErrAng(float posicao[2], nav_msgs::Odometry retorno, float anglo)
{
	float err=99;
	err = (atan2(posicao[1]-retorno.pose.pose.position.y,posicao[0]-retorno.pose.pose.position.x))-anglo;
	if(err<=-3.14) err = 6.28+err;
	if(err>=3.14) err = err-6.28;
	return err;
}

void movFixRobo(float vel,geometry_msgs::Twist msg,ros::Publisher publicador)
{
	msg.linear.x = vel;
	msg.angular.z = 0;
	publicador.publish(msg);
}

float controlePos(float posdesejada[2], float oridesejada, nav_msgs::Odometry feedback, ros::Publisher publicador, sensor_msgs::LaserScan sensor, float angule, int rob)
{
	geometry_msgs::Twist msg;
	float min_dist = 99, erroLin = 99, erroAng = 99;
	float angulo=0, velLin = 0 , velAng = 0, valordavez=99;
	float desvLin = 0, desvAng = 0, areaDmin = 0, areaCmin=0, areaEmin=0;
	vector<float> areaD, areaC, areaE;

	ROS_INFO("NAV %i--------------------------------------",rob);

	if(oridesejada != 0)
	{
		erroAng = oridesejada-angule;
		if(erroAng<=-3.14) erroAng = 6.28+erroAng;
		if(erroAng>=3.14) erroAng = erroAng-6.28;
		velAng = fcNavAng.make_inference(erroAng);
		msg.linear.x = 0;
		msg.angular.z = 1*velAng;
		ROS_WARN("Arrumando...");
		valordavez=erroAng;
	}
	else
	{
		erroLin = calcErrLin(posdesejada,feedback);
		velLin = fcNavLin.make_inference(erroLin);
		msg.linear.x = velLin;

		erroAng = calcErrAng(posdesejada,feedback,angule);
		velAng = fcNavAng.make_inference(erroAng);
		msg.angular.z = 3*velAng;

		areaD={sensor.ranges.begin(),sensor.ranges.begin()+7};
		areaDmin=*min_element(areaD.begin(),areaD.end());
		areaC={sensor.ranges.begin()+8,sensor.ranges.begin()+16};
		areaCmin=*min_element(areaC.begin(),areaC.end());
		areaE={sensor.ranges.end()-8,sensor.ranges.end()};
		areaEmin=*min_element(areaE.begin(),areaE.end());

		min_dist = *min_element(sensor.ranges.begin(),sensor.ranges.end());

		if ( min_dist <=0.4)
		{
			desvLin=fcDesvLin.make_inference(min_dist);
			desvAng=fcDesvAng.make_inference(areaEmin,areaCmin,areaDmin);
			msg.linear.x = desvLin;
			msg.angular.z = 0.5*desvAng;
			ROS_WARN("Desvio>> LIN: %f ANG: %f",desvLin,desvAng);
		}
		else
		{
		     	ROS_INFO("Atual>>X:%f,Y:%f,ANG:%f", feedback.pose.pose.position.x, feedback.pose.pose.position.y, angule);
			ROS_INFO("Alvo >>X:%f,Y:%f,ANG:%f",posdesejada[0], posdesejada[1], erroAng+angule);
		 	ROS_INFO("IN -> LIN>>%f ANG>>%f",erroLin,erroAng);
			ROS_INFO("OUT -> LIN>>%f ANG>>%f",velLin,velAng);
			ROS_INFO("DESV-------------------------------------");
			ROS_INFO("Sensor>>E:%f C:%f D:%f", areaEmin, areaCmin, areaDmin);
			ROS_INFO("------------------------------------------");
		}
		valordavez=erroLin;		
	}

	publicador.publish(msg);
	
	return valordavez;
}

void subCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	feedback.pose.pose.position.x = msg->pose.pose.position.x;
	feedback.pose.pose.position.y = msg->pose.pose.position.y;
	feedback.pose.pose.position.z = 0;
	feedback.pose.pose.orientation.x = msg->pose.pose.orientation.x;
	feedback.pose.pose.orientation.y = msg->pose.pose.orientation.y;
	feedback.pose.pose.orientation.z = msg->pose.pose.orientation.z;
	feedback.pose.pose.orientation.w = msg->pose.pose.orientation.w;

	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
  	yaw_angle = (tf::getYaw(pose.getRotation())*-1)-(1.57079);
	if(yaw_angle<=-3.14) yaw_angle=6.28+yaw_angle;
}

void subCallback2(const nav_msgs::Odometry::ConstPtr& msg2)
{
	feedback2.pose.pose.position.x = msg2->pose.pose.position.x;
	feedback2.pose.pose.position.y = msg2->pose.pose.position.y;
	feedback2.pose.pose.position.z = 0;
	feedback2.pose.pose.orientation.x = msg2->pose.pose.orientation.x;
	feedback2.pose.pose.orientation.y = msg2->pose.pose.orientation.y;
	feedback2.pose.pose.orientation.z = msg2->pose.pose.orientation.z;
	feedback2.pose.pose.orientation.w = msg2->pose.pose.orientation.w;

	tf::Pose pose;
	tf::poseMsgToTF(msg2->pose.pose, pose);
  	yaw_angle2 = (tf::getYaw(pose.getRotation())*-1)-(1.57079);
	if(yaw_angle2<=-3.14) yaw_angle2=6.28+yaw_angle2;
}

void subSense(const sensor_msgs::LaserScan::ConstPtr& sns)
{
	sensor.ranges = sns->ranges;
}

void subSense2(const sensor_msgs::LaserScan::ConstPtr& sns2)
{
	sensor2.ranges = sns2->ranges;
}

void subGarra(const std_msgs::Float32 gr)
{
	flagG=gr.data;
}
void subGarra2(const std_msgs::Float32 gr2)
{
	flagG2=gr2.data;
}

void setMaq(int maq, int robo, float posicao[2])
{
	if (robo == 1)
	{
		switch (maq)
		{
			case 1:
				posicao[0]=-2.7;
				posicao[1]=1.8;			
			break;
			case 2:
				posicao[0]=-1.95;
				posicao[1]=1.8;			
			break;
			case 3:
				posicao[0]=-1.3;
				posicao[1]=1.8;			
			break;
			case 4:
				posicao[0]=-0.65;
				posicao[1]=1.8;			
			break;
			case 5:
				posicao[0]=0.05;
				posicao[1]=1.8;			
			break;
			case 6:
				posicao[0]=0.7;
				posicao[1]=1.8;			
			break;
		}
	}
	else
	{
		switch (maq)
		{
			case 1:
				posicao[0]=-0.7;
				posicao[1]=2.3;			
			break;
			case 2:
				posicao[0]=0;
				posicao[1]=2.3;			
			break;
			case 3:
				posicao[0]=0.68;
				posicao[1]=2.3;			
			break;
			case 4:
				posicao[0]=1.3;
				posicao[1]=2.3;			
			break;
			case 5:
				posicao[0]=2.;
				posicao[1]=2.3;			
			break;
			case 6:
				posicao[0]=2.7;
				posicao[1]=2.3;			
			break;
		}
	}
}

//---controles FUZZY---//
void startFcDesvLin()
{
	fuzzy_set catLin[10];
	linguisticvariable proximidade, coragem;
	rule regrasDesvLin[2];

	catLin[0].setname("perto"); 
	catLin[0].setrange(-15,+15); 
	catLin[0].setval(0,0,0.1,0.3);
	catLin[1].setname("longe"); 
	catLin[1].setrange(-15,15); 
	catLin[1].setval(0.1,0.3,0.5,0.5);
	proximidade.setname("proximidade");
	proximidade.includecategory(&catLin[0]);
	proximidade.includecategory(&catLin[1]);

	catLin[5].setname("para"); 
	catLin[5].setrange(-1,+1); 
	catLin[5].setval(-1,-1,-0.5,-0.05);
	catLin[6].setname("vai"); 
	catLin[6].setrange(-1,+1); 
	catLin[6].setval(-0.61,-0.3,0,0);
	coragem.setname("coragem");
	coragem.includecategory(&catLin[5]);
	coragem.includecategory(&catLin[6]);

	fcDesvLin.set_defuzz(CENTROID);
	fcDesvLin.definevars(proximidade, coragem);

	fcDesvLin.insert_rule("perto","para");
	fcDesvLin.insert_rule("longe","vai");

	fcDesvLin.save_m("vaiounao", 0);
}
void startFcDesvAng()
{
	fuzzy_set catAng[9];
	linguisticvariable distanciaESQ, distanciaDIR, distanciaCENT, velocidade;
	rule regrasDesvAng[6];

	catAng[0].setname("pertoE"); 
	catAng[0].setrange(-15,15); 
	catAng[0].setval(0,0,0.2,0.4);
	catAng[1].setname("longeE"); 
	catAng[1].setrange(-15,15); 
	catAng[1].setval(0.2,0.4,15,15);
	distanciaESQ.setname("distanciaESQ");
	distanciaESQ.includecategory(&catAng[0]);
	distanciaESQ.includecategory(&catAng[1]);

	catAng[2].setname("pertoD"); 
	catAng[2].setrange(-15,15); 
	catAng[2].setval(0,0,0.2,0.4);
	catAng[3].setname("longeD"); 
	catAng[3].setrange(-15,15); 
	catAng[3].setval(0.2,0.4,15,15);
	distanciaDIR.setname("distanciaDIR");
	distanciaDIR.includecategory(&catAng[2]);
	distanciaDIR.includecategory(&catAng[3]);

	catAng[4].setname("pertoCENT"); 
	catAng[4].setrange(-15,15); 
	catAng[4].setval(0,0,0.5,1);
	catAng[5].setname("longeCENT"); 
	catAng[5].setrange(-15,15); 
	catAng[5].setval(0.5,1,15,15);
	distanciaCENT.setname("distanciaCENT");
	distanciaCENT.includecategory(&catAng[4]);
	distanciaCENT.includecategory(&catAng[5]);

	catAng[6].setname("VD"); 
	catAng[6].setrange(-2,+2); 
	catAng[6].setval(-2,-2,-1,-0.5);
	catAng[7].setname("SR"); 
	catAng[7].setrange(-2,+2); 
	catAng[7].setval(-1,-0.5,0.5,1);
	catAng[8].setname("VE"); 
	catAng[8].setrange(-2,+2); 
	catAng[8].setval(0.5,1,2,2);
	velocidade.setname("velocidade");
	velocidade.includecategory(&catAng[6]);
	velocidade.includecategory(&catAng[7]);
	velocidade.includecategory(&catAng[8]);

	fcDesvAng.set_defuzz(CENTROID);
	fcDesvAng.definevars(distanciaESQ, distanciaCENT, distanciaDIR, velocidade);

	fcDesvAng.insert_rule("longeE","longeCENT","longeD","SR");
	fcDesvAng.insert_rule("longeE","longeCENT","pertoD","VD");
	fcDesvAng.insert_rule("longeE","pertoCENT","longeD","VE");
	fcDesvAng.insert_rule("longeE","pertoCENT","pertoD","VD");
	fcDesvAng.insert_rule("pertoE","longeCENT","longeD","VE");
	fcDesvAng.insert_rule("pertoE","longeCENT","pertoD","SR");
	fcDesvAng.insert_rule("pertoE","pertoCENT","longeD","VE");
	fcDesvAng.insert_rule("pertoE","pertoCENT","pertoD","VE");

	fcDesvAng.save_m("viraounao", 0);
}

void startFcNavLin()
{
	fuzzy_set cat[6];
	linguisticvariable distancia, velocidade, aceleracao;
	rule regrasNavLin[6];

	cat[0].setname("perto"); 
	cat[0].setrange(-5,+5); 
	cat[0].setval(0,0,0.5,1);
	cat[1].setname("medio"); 
	cat[1].setrange(-5,+5); 
	cat[1].setval(0.5,1,2,3);
	cat[2].setname("longe"); 
	cat[2].setrange(-5,+5); 
	cat[2].setval(2,3,5,5);
	distancia.setname("distancia");
	distancia.includecategory(&cat[0]);
	distancia.includecategory(&cat[1]);
	distancia.includecategory(&cat[2]);

	cat[3].setname("reduz"); 
	cat[3].setrange(-1,+1); 
	cat[3].setval(0.1,0.1,0.3,0.5);
	cat[4].setname("mantem"); 
	cat[4].setrange(-1,+1); 
	cat[4].setval(0.3,0.7,0.8);
	cat[5].setname("acelera"); 
	cat[5].setrange(-1,+1); 
	cat[5].setval(0.6,0.75,1,1);
	aceleracao.setname("aceleracao");
	aceleracao.includecategory(&cat[3]);
	aceleracao.includecategory(&cat[4]);
	aceleracao.includecategory(&cat[5]);

	fcNavLin.set_defuzz(CENTROID);
	fcNavLin.definevars(distancia, aceleracao);

	fcNavLin.insert_rule("perto","reduz");
	fcNavLin.insert_rule("medio","mantem");
	fcNavLin.insert_rule("longe","acelera");

	fcNavLin.save_m("controleteste", 0);
}

void startFcNavAng()
{
	fuzzy_set cat2[10];
	linguisticvariable diferenca, rotacao;
	rule regrasNavLin[5];

	cat2[0].setname("GN"); 
	cat2[0].setrange(-PI,+PI); 
	cat2[0].setval(-PI,-PI,-PI/2,-PI/3);
	cat2[1].setname("PN"); 
	cat2[1].setrange(-PI,+PI); 
	cat2[1].setval(-PI/2,-PI/4,0);
	cat2[2].setname("quaseNada"); 
	cat2[2].setrange(-PI,+PI); 
	cat2[2].setval(-0.3*PI,-PI/8,PI/8,0.3*PI);
	cat2[3].setname("PP"); 
	cat2[3].setrange(-PI,+PI); 
	cat2[3].setval(0,PI/4,PI/2);
	cat2[4].setname("GP"); 
	cat2[4].setrange(-PI,+PI); 
	cat2[4].setval(PI/3,PI/2,PI,PI);
	diferenca.setname("diferenca");
	diferenca.includecategory(&cat2[0]);
	diferenca.includecategory(&cat2[1]);
	diferenca.includecategory(&cat2[2]);
	diferenca.includecategory(&cat2[3]);
	diferenca.includecategory(&cat2[4]);

	cat2[5].setname("rapidaNEG"); 
	cat2[5].setrange(-2,2); 
	cat2[5].setval(-2,-2,-1.5,-1);
	cat2[6].setname("devagarNEG"); 
	cat2[6].setrange(-2,2); 
	cat2[6].setval(-1.5,-0.5,0);
	cat2[7].setname("para"); 
	cat2[7].setrange(-2,2); 
	cat2[7].setval(-0.5,0,0.5);
	cat2[8].setname("devagarPOS"); 
	cat2[8].setrange(-2,2); 
	cat2[8].setval(0,0.5,1.5);
	cat2[9].setname("rapidaPOS"); 
	cat2[9].setrange(-2,2); 
	cat2[9].setval(1,1.5,2,2);
	rotacao.setname("rotacao");
	rotacao.includecategory(&cat2[5]);
	rotacao.includecategory(&cat2[6]);
	rotacao.includecategory(&cat2[7]);
	rotacao.includecategory(&cat2[8]);
	rotacao.includecategory(&cat2[9]);

	fcNavAng.set_defuzz(CENTROID);
	fcNavAng.definevars(diferenca, rotacao);

	fcNavAng.insert_rule("PN","devagarNEG");
	fcNavAng.insert_rule("GN","rapidaNEG");
	fcNavAng.insert_rule("PP","devagarPOS");
	fcNavAng.insert_rule("GP","rapidaPOS");
	fcNavAng.insert_rule("quaseNada","para");

	fcNavAng.save_m("controletesteANG", 0);
}
