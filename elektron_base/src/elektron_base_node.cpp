#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <pthread.h>
#include "serialcomm/serialcomm.hpp"
#include "nf/nfv2.h"
#include "math.h"
#include <mutex>

/**
Spis treści:
-definicje, zmienne i obiekty globalne
-ReadDeviceVitals	-konstruowanie polecenia przesłania danych przez sterownik napędów.
-VelocityCommandCallback	-obsługa wiadomości przychodzących na topic.
-IncomingDataListener	-nasłuchiwanie na dane przychodzące od sterownika napędów.
-TimeoutCheck	-zabezpieczenie na brak nowych poleceń.
-SendOrders	-wysyłanie poleceń do sterownika napędów.
-OdometryCalculation	-obliczanie odometrii.
-Initialization	-inicjalizacja obiektów i zmiennych globalnych.
-Finish	-zwalnianie wskaźników.
-main	-główna funkcja programu.
*/

//Domyślne wartości parametrów robota i sterownika.
#define DEFAULT_DRIVES_DRIVER_ADDRESS "/dev/ttyUSB0"
#define DEFAULT_SEND_LOOP_EXECUTE_RATE_VALUE 200
#define DEFAULT_READ_DEVICE_VITALS_RATE_VALUE 10
#define DEFAULT_AXLE_LENGTH 0.314
#define DEFAULT_ENCODER_TICKS 5000
#define DEFAULT_WHEEL_DIAMETER 0.1
#define DEFAULT_REGULATOR_RATE 100
#define DEFAULT_COMMAND_TIMEOUT_TIME 3
#define DEFAULT_TIMEOUT_COMMAND_SEND_TIME 1
#define DEFAULT_PUBLISH_ODOMETRY_TF true

//Parametry robota i sterownika, pobierane z launchfile.
std::string drivesDriverAddress;	//Adres płytki sterownika silników.
double sendLoopExecuteRateValue;	//Częstotliwość wywoływania pętli wysyłania poleceń do płytki.
double readDeviceVitalsRateValue;	//Częstotliwość konstruowania się polecenia przesłania danych przez płytkę.
double axleLength;	//Długość osi robota [m].
double encoderTicks;	//Ilość tick'ów enkodera uzyskanych w jednym obrocie koła (Impulsów będzie 4 razy tyle).
double wheelDiameter;	//Średnica koła robota [m].
double regulatorRate;	//Częstotliwość pracy regulatora [Hz].
double commandTimeoutTime;	//Czas po którym następuje zatrzymanie silników w przypadku braku otrzymania nowego polecenie prędkości.
double timeoutCommandSendTime;	//Czas co który następuje wysyłanie polecenia zatrzymania silników przez timeout.

//Obiekty do komunikacji i pomocnicze.
pthread_t listenerThread;	//Wątek zbierający dane przychodzące od płytki.
ros::Timer* readDeviceVitalsRate;	//Zegar dodający polecenie podania danych z płytki.
ros::NodeHandle* nodeHandlePublic;	//Node handle z nazwą w przestrzeni publicznej
ros::NodeHandle* nodeHandlePrivate;	//Node handle z nazwą w przestrzeni prywatnej
ros::Subscriber* velocitySubscriber;	//Subscriber odbierający polecenia sterowania prędkością przychodzące na topic.

//Obiekty i struktury biblioteki NFv2
NF_STRUCT_ComBuf NFCommunicationBuffer;	//Bufor komunikacyjny biblioteki NFv2.
SerialComm *communicationPort;	//Port komunikacyjny.
uint8_t txBuffer[256];	//Dane wysylane do plytki.
uint8_t txCount;		//Dlugosc danych wysylanych do plytki.
uint8_t commandArray[256];	//Tablica polecen do sterownika.
uint8_t commandCount;		//Stopien zapelnienia tablicy polecen do sterownika.
uint8_t rxCommandArray[256];	//Tablica danych przychodzących od płytki.
uint8_t rxCommandCount;	//Stopien zapelnienia tablicy danych od płytki.
uint8_t rxBuffer[256];	//Dane odebrane do plytki.
uint8_t rxCount;		//Dlugosc danych odebranych od plytki.

//Zmienne i struktury odometrii.
ros::Publisher* odometryPublisher;	//Publisher odometrii.
tf::TransformBroadcaster* odometryBroadcaster;	//Broadcaster odometrii.
nav_msgs::Odometry* odometryMessage;	//Wiadomość o odometrii.
geometry_msgs::TransformStamped* odometryTransform;	//Transformata odomerii.
double x=0;	//Pozycja robota na osi X.
double x_old=0;	//Poprzednia pozycja robota na osi X.
double y=0;	//Pozycja robota na osi Y.
double y_old=0;	//Poprzednia pozycja robota na osi Y.
double th=0;	//Orientacja robota.
double th_old=0;	//Poprzednia orientacja robota.
double linearDistance;	//Dystans liniowy przebyty przez robota.
double x_vel=0;	//Prędkość liniowa robota.
double th_vel=0;	//Prędkość kątowa.
double delta_l=0;	//Zmiana ilości impulsów lewego koła.
double delta_r=0;	//Zmiana ilości impulsów prawego koła.
double r_pos_total_now=0;	//Aktualna przeliczona pozycja kola prawego w SI [m].
double l_pos_total_now=0;	//Aktualna przeliczona pozycja kola lewego w SI [m].
uint32_t r_pos_total=0;		//Aktualna pobrana pozycja kola prawego.
uint32_t l_pos_total=0; 	//Aktualna pobrana pozycja kola lewego.
double r_pos_total_old=0;	//Poprzednia zbiorcza pozycja kola prawego.
double l_pos_total_old=0; 	//Poprzednia zbiorcza pozycja kola lewego.
ros::Time last_time(0);		//Moment ostatniego pomiaru.
ros::Time current_time(0);	//Aktualny moment pomiaru.
bool publishOdometryTF;		//Czy publikować odomoetrię na TF.
bool firstOdometryCalculation = true;	//Czy jest to pierwsze przejście obliczania odometrii.

//Globalne zmienne pomocnicze programu.
std::mutex commandArrayMutex;	//Mutex nadzorujący dostęp do CommandArray.
bool drivesCommandChange = false;	//Określa czy przyszło nowe polecenie prędkości.
bool readCommandAdded = false;	//Określa czy w danych do wysłania znajduje się już polecenie podania danych przez płytkę.
double commandTimeoutCounter = 0;	//Odmierza czas od otrzymania ostatniego polecenia sterowania.
double timeoutCommandCooldown = 0;	//Odmierza czas od wysłania polecenia zatrzymania silników z uwagi na timeout. 


/**
	Konstruuje polecenie podania danych przez płytkę.
	Uruchamiany okresowo przez zegar readDeviceVitalsRate.
*/
void ReadDeviceVitals(const ros::TimerEvent&) 
{
	if(!readCommandAdded)
	{
		std::lock_guard<std::mutex> lock(commandArrayMutex);
		commandArray[commandCount++] = NF_COMMAND_ReadDeviceVitals;
		commandArray[commandCount++] = NF_COMMAND_ReadDrivesPosition;
		readCommandAdded = true;
	}
}


/**
	Odbiera wiadomość z topic'a i wylicza zadane prędkości dla silników.
	Uruchamiany w wyniku otrzymania wiadomości na topic.
*/
void VelocityCommandCallback(const geometry_msgs::TwistConstPtr& msg) 
{
	//Wyliczenie proporcji skretu do ruchu postepowego.
	double rotation = msg->angular.z * axleLength / 2.0;

	//Wyliczenie predkosci poszczegolnych kol.
	double 	leftWheelVelocity = ((msg->linear.x - rotation) * encoderTicks) / (2 * 3.14 * wheelDiameter * regulatorRate);
	double	rightWheelVelocity = ((msg->linear.x + rotation) * encoderTicks) / (2 * 3.14 * wheelDiameter * regulatorRate);

	std::lock_guard<std::mutex> lock(commandArrayMutex);
	NFCommunicationBuffer.SetDrivesSpeed.data[0] = leftWheelVelocity;
	NFCommunicationBuffer.SetDrivesSpeed.data[1] = rightWheelVelocity;
	
	NFCommunicationBuffer.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
	NFCommunicationBuffer.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
	
	commandTimeoutCounter = 0;	//Zerowanie licznika po otrzymaniu polecenia.
	timeoutCommandCooldown = 0;	
	drivesCommandChange = true;	
}


/**
	Odbiera i interpretuje dane przychodzące od płytki.
	Uruchamiany jako oddzielny wątek.
*/
void *IncomingDataListener(void *p)
{
	while(1) 
	{
		if(rxCount == 255)
		{
			rxCount = 0;
		}
		rxBuffer[rxCount] = communicationPort->readOneByte();
		if(NF_Interpreter(&NFCommunicationBuffer, rxBuffer, &rxCount, rxCommandArray, &rxCommandCount) > 0)
		{
			if(NFCommunicationBuffer.ReadDrivesPosition.updated)
			{
				l_pos_total = NFCommunicationBuffer.ReadDrivesPosition.data[0]; 
				r_pos_total = NFCommunicationBuffer.ReadDrivesPosition.data[1];
				NFCommunicationBuffer.ReadDrivesPosition.updated=0;
				ROS_INFO("Wheels position. Left: %d; Right: %d;", NFCommunicationBuffer.ReadDrivesPosition.data[0], NFCommunicationBuffer.ReadDrivesPosition.data[1]);
			}
			current_time = ros::Time::now();
		}		
	}
	pthread_exit(NULL);
}


/**
	Sprawdza czy został przekroczony czas podczas którego miało przyjść polecenie prędkości na topic.
	Jeśli tak, to następuje dodanie polecenia zatrzymania robota.
*/
void TimeoutCheck()
{
	std::lock_guard<std::mutex> lock(commandArrayMutex);
	if(commandTimeoutCounter >= commandTimeoutTime)
	{
		if(timeoutCommandCooldown <= 0)
		{
			NFCommunicationBuffer.SetDrivesSpeed.data[0] = 0;
			NFCommunicationBuffer.SetDrivesSpeed.data[1] = 0;
			NFCommunicationBuffer.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
			NFCommunicationBuffer.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
			timeoutCommandCooldown = timeoutCommandSendTime;
			drivesCommandChange = true;
			ROS_WARN("Drives stopped due to timeout.");
		}
		else
		{
			timeoutCommandCooldown -= (1/sendLoopExecuteRateValue);
		}
	}
	else
	{
		commandTimeoutCounter += (1/sendLoopExecuteRateValue);
	}
}


/**
	Konstruuje pakiet komunikacyjny.
*/
void SendOrders()
{
	std::lock_guard<std::mutex> lock(commandArrayMutex);
	if(drivesCommandChange)
	{
		commandArray[commandCount++] = NF_COMMAND_SetDrivesMode;
		commandArray[commandCount++] = NF_COMMAND_SetDrivesSpeed;
		drivesCommandChange = false;
	}
	if(commandCount > 0) 
	{
		txCount = NF_MakeCommandFrame(&NFCommunicationBuffer, txBuffer, (const uint8_t*)commandArray, commandCount, NF_RobotAddress);
		commandCount = 0;
		communicationPort->write(txBuffer, txCount);
		readCommandAdded = false;
	}
}


/**
	Oblicza i publikuje odometrie robota.
*/
void OdometryCalculation()
{
	//Konwersja danych timerów na sekundy.
	double secs_now = current_time.toSec();
	double secs_last = last_time.toSec();
		
	//Wyliczenie roznicy przebytych drog dla poszczegolnych kol.
	r_pos_total_now = (double)((0.314*r_pos_total)*0.00005);
	l_pos_total_now = (double)((0.314*l_pos_total)*0.00005);
				
	//Jesli nastąpi nieoczekiwany skok wartosci ktoregokolwiek licznika enkodera to nadpisz poprzednia wartosc nowa wartoscia.	
	if (l_pos_total_now>l_pos_total_old+1 || l_pos_total_now<l_pos_total_old-1 || r_pos_total_now>r_pos_total_old+1 || r_pos_total_now<r_pos_total_old-1 || firstOdometryCalculation)
	{
		r_pos_total_old = r_pos_total_now;
		l_pos_total_old = l_pos_total_now;
		firstOdometryCalculation = false;
	}

	delta_r = r_pos_total_now - r_pos_total_old;
	delta_l = l_pos_total_now - l_pos_total_old;

	//Wyznaczenie pozycji.
	th = th_old - ((delta_l-delta_r) / axleLength);	//Całkowity kąt obrotu.
	linearDistance = (delta_l+delta_r) / 2;
	x = x_old + linearDistance * cos(th);
	y= y_old + linearDistance * sin(th);

	if(secs_now-secs_last > 0.001 || -0.001 > secs_now-secs_last)
	{
		th_vel = (th-th_old) / (secs_now-secs_last);
		x_vel = linearDistance / (secs_now-secs_last);
	}
	else
	{
		th_vel=0;
		x_vel=0;
	}

	//Wyliczenie kwaternionu skrętu.
	geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

	if (publishOdometryTF) 
	{
		//Publikacja transformaty na TF.
		odometryTransform->header.stamp = current_time;
				
		odometryTransform->transform.translation.x = x;
		odometryTransform->transform.translation.y = y;
		odometryTransform->transform.translation.z = 0.0;
		odometryTransform->transform.rotation = odometryQuaternion;

		//Wysłanie transformaty.
		odometryBroadcaster->sendTransform(*odometryTransform);
	}

	//Publikowanie odometrii w ROS.
	odometryMessage->header.stamp = current_time;

	//Wyznaczanie pozycji.
	odometryMessage->pose.pose.position.x = x;
	odometryMessage->pose.pose.position.y = y;
	odometryMessage->pose.pose.position.z = 0.0;
	odometryMessage->pose.pose.orientation = odometryQuaternion;

	odometryMessage->pose.covariance[0] = 0.00001;
	odometryMessage->pose.covariance[7] = 0.00001;
	odometryMessage->pose.covariance[14] = 10.0;
	odometryMessage->pose.covariance[21] = 1.0;
	odometryMessage->pose.covariance[28] = 1.0;
	odometryMessage->pose.covariance[35] = th_vel + 0.001;

	//Określanie prędkości.
	odometryMessage->child_frame_id = "base_link";
	odometryMessage->twist.twist.linear.x = x_vel;
	odometryMessage->twist.twist.linear.y = 0.0;
	odometryMessage->twist.twist.angular.z = th_vel;

	//Publikowanie wiadomości.
	odometryPublisher->publish(*odometryMessage);
			

	//Przesłanie nowych wartości do bufora.
	last_time = current_time;

	x_old = x;
	y_old = y;
	th_old = th;
			
	r_pos_total_old = r_pos_total_now;  
	l_pos_total_old = l_pos_total_now;
}


/**
	Inicializuje parametry i obiekty wykorzystywane w programie.
	Zwraca "true" w przypadku powodzenia, a "false" w przypadku porażki.
*/
bool Initialization(int argc, char** argv)
{
	std::lock_guard<std::mutex> lock(commandArrayMutex);
	bool success = true;

	ros::init(argc, argv, "elektron_base_node");	//Inicializacja ROS.
	nodeHandlePublic = new ros::NodeHandle();	//Utworzenie node handle z nazwą publiczną.
	nodeHandlePrivate = new ros::NodeHandle("~");	//Utworzenie node handle z nazwą prywatną.

	//Inicializacja wszystkich parametrów, zmiana wartości zmiennej success z true na false w przypadku błędów.
	if (!nodeHandlePrivate->getParam("drivesDriverAddress", drivesDriverAddress)) 
	{
		drivesDriverAddress = DEFAULT_DRIVES_DRIVER_ADDRESS;
		ROS_INFO("Using default drivesDriverAddress value: %s.", drivesDriverAddress.c_str());
	}
	if (!nodeHandlePrivate->getParam("sendLoopExecuteRate", sendLoopExecuteRateValue))
	{
		sendLoopExecuteRateValue = DEFAULT_SEND_LOOP_EXECUTE_RATE_VALUE;
		ROS_INFO("Using default sendLoopExecuteValue value: %f.", sendLoopExecuteRateValue);
	}
	if (!nodeHandlePrivate->getParam("readDeviceVitalsRate", readDeviceVitalsRateValue))
	{
		readDeviceVitalsRateValue = DEFAULT_READ_DEVICE_VITALS_RATE_VALUE;
		ROS_INFO("Using default readDeviceVitalsRateValue value: %f.", readDeviceVitalsRateValue);
	}
	if (!nodeHandlePrivate->getParam("axleLength", axleLength)) 
	{
		axleLength = DEFAULT_AXLE_LENGTH;
		ROS_INFO("Using default axleLength value: %f.", axleLength);
	}
	if (!nodeHandlePrivate->getParam("encoderTicks", encoderTicks)) 
	{
		encoderTicks = DEFAULT_ENCODER_TICKS;
		ROS_INFO("Using default encoderTicks value: %f.", encoderTicks);
	}
	if (!nodeHandlePrivate->getParam("wheelDiameter", wheelDiameter)) 
	{
		wheelDiameter = DEFAULT_WHEEL_DIAMETER;
		ROS_INFO("Using default wheelDiameter value: %f.", wheelDiameter);
	}
	if (!nodeHandlePrivate->getParam("regulatorRate", regulatorRate)) 
	{
		regulatorRate = DEFAULT_REGULATOR_RATE;
		ROS_INFO("Using default regulatorRate value: %f.", regulatorRate);
	}
	if (!nodeHandlePrivate->getParam("commandTimeoutTime", commandTimeoutTime)) 
	{
		commandTimeoutTime = DEFAULT_COMMAND_TIMEOUT_TIME;
		ROS_INFO("Using default commandTimeoutTime value: %f.", commandTimeoutTime);
	}
	if (!nodeHandlePrivate->getParam("timeoutCommandSendTime", timeoutCommandSendTime)) 
	{
		timeoutCommandSendTime = DEFAULT_TIMEOUT_COMMAND_SEND_TIME;
		ROS_INFO("Using default timeoutCommandSendTime value: %f.", timeoutCommandSendTime);
	}
	if (!nodeHandlePrivate->getParam("publishOdometryTF", publishOdometryTF)) 
	{
		publishOdometryTF = DEFAULT_PUBLISH_ODOMETRY_TF;
		ROS_INFO("Using default publishOdometryTF value: %d.", publishOdometryTF);
	}

	//Inicjalizacja biblioteki NFv2
	NFv2_Config(&NFCommunicationBuffer, NF_TerminalAddress);
        communicationPort = new SerialComm(drivesDriverAddress);	
	if (!(communicationPort->isConnected())) 
	{
		ROS_ERROR("NFv2 connection to device %s failed.", drivesDriverAddress.c_str());
		success = false;
	}
	
	odometryPublisher = new ros::Publisher(nodeHandlePublic->advertise<nav_msgs::Odometry> ("odom", 1));
	odometryBroadcaster = new tf::TransformBroadcaster();
	odometryMessage = new nav_msgs::Odometry();
	odometryTransform = new geometry_msgs::TransformStamped();

	odometryMessage->header.frame_id = "odom";
	odometryMessage->child_frame_id = "base_link";
	odometryTransform->header.frame_id = "odom";
	odometryTransform->child_frame_id = "base_link";	

	velocitySubscriber = new ros::Subscriber(nodeHandlePublic->subscribe("cmd_vel", 1, &VelocityCommandCallback));

	if(pthread_create(&listenerThread, NULL, &IncomingDataListener, NULL))
	{
		ROS_ERROR("Failed to start incoming data listener thread.");
		success = false;
	}

	readDeviceVitalsRate = new ros::Timer(nodeHandlePublic->createTimer(ros::Duration(1/readDeviceVitalsRateValue), ReadDeviceVitals));

	return success;
}


/**
	Zwalnia używane wskaźniki.
*/
void Finish()
{
	delete odometryPublisher;
	delete odometryBroadcaster;
	delete odometryMessage;
	delete odometryTransform;
	delete readDeviceVitalsRate;
	delete velocitySubscriber;
	delete nodeHandlePublic;
	delete nodeHandlePrivate;
}


/**
	Główna funkcja programu.
*/
int main(int argc, char** argv)
{
	if(Initialization(argc, argv))
	{
		ROS_INFO("Initialization sucessfull.");
		ros::Rate sendLoopExecuteRate(sendLoopExecuteRateValue);
		while (ros::ok()) 
		{
			TimeoutCheck();
			SendOrders();
			OdometryCalculation();
			ros::spinOnce();
			sendLoopExecuteRate.sleep();
			if(ros::isShuttingDown())
			{
				Finish();
				break;
			}
		}
		return 0;
	}
	else
	{
		ROS_ERROR("Initialization failed.");
		Finish();
		return 1;
	}
}
