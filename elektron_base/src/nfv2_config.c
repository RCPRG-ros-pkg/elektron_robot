#include "stdio.h"
#include "nf/nfv2.h"

void NFv2_Config(NF_STRUCT_ComBuf *NFComBuf, uint8_t myAddress){

	NFComBuf->myAddress = myAddress;
	NFComBuf->dataReceived = 0;

	// Address map is essential only when device acts as Master on NFv2 bus.
	
	NFComBuf->myAddress = NF_TerminalAddress;
	NFComBuf->ReadDeviceVitals.addr[0] = NF_RobotAddress;
	NFComBuf->ReadDeviceVitals.addr[1] = NF_RobotAddress;
	NFComBuf->ReadDeviceVitals.addr[2] = NF_RobotAddress;
	NFComBuf->ReadDeviceVitals.addr[3] = NF_RobotAddress;
	NFComBuf->ReadDeviceVitals.addr[4] = NF_RobotAddress;
	NFComBuf->ReadDeviceVitals.addr[5] = NF_RobotAddress;
	NFComBuf->ReadDeviceVitals.addr[6] = NF_RobotAddress;
	NFComBuf->ReadDeviceVitals.addr[7] = NF_RobotAddress;
	NFComBuf->SetDrivesMode.addr[0] = NF_RobotAddress;
	NFComBuf->SetDrivesMode.addr[1] = NF_RobotAddress;
	NFComBuf->SetDrivesSpeed.addr[0] = NF_RobotAddress;
	NFComBuf->SetDrivesSpeed.addr[1] = NF_RobotAddress;
	NFComBuf->SetDigitalOutputs.addr[0] = NF_RobotAddress;

	NFComBuf->ReadDrivesPosition.addr[0] = NF_RobotAddress;
	NFComBuf->ReadDrivesPosition.addr[1] = NF_RobotAddress;
	
	NFv2_CrcInit();
}
