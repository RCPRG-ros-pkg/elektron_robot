/*
* External config file.
* Uncomment and update buffers' size [no of param set instances] for commands
* to be supported by module:
*/

#define NF_BroadcastAddress 0
#define NF_RobotAddress     0x01
#define NF_TerminalAddress  123

/*
* Uncomment and update buffers' size [no of param set instances] for commands
* to be supported by module:
*/
//	#define NF_BUFSZ_ReadDeviceStatus	1
    #define NF_BUFSZ_ReadDeviceVitals	8

    #define NF_BUFSZ_SetDrivesMode		2
    #define NF_BUFSZ_SetDrivesSpeed		2
//	#define NF_BUFSZ_SetDrivesCurrent	2
//	#define NF_BUFSZ_SetDrivesPosition	2

//	#define NF_BUFSZ_SetServosMode		4
//  #define NF_BUFSZ_SetServosPosition	4
//	#define NF_BUFSZ_SetServosSpeed		4

    #define NF_BUFSZ_SetDigitalOutputs	1
  #define NF_BUFSZ_ReadAnalogInputs	8

/*
* Remember to declare:
* extern NF_STRUCT_ComBuf	NFComBuf;
*/
