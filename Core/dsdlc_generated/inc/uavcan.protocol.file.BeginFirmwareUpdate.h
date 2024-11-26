

#pragma once
#include "../inc/uavcan.protocol.file.BeginFirmwareUpdate_req.h"
#include "../inc/uavcan.protocol.file.BeginFirmwareUpdate_res.h"

#define UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID 40
#define UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE (0xB7D725DF72724126ULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(uavcan_protocol_file_BeginFirmwareUpdate, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_REQUEST_MAX_SIZE, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE);
#endif
