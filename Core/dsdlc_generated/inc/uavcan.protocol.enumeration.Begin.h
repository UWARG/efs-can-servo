

#pragma once
#include "../inc/uavcan.protocol.enumeration.Begin_req.h"
#include "../inc/uavcan.protocol.enumeration.Begin_res.h"

#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_ID 15
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_SIGNATURE (0x196AE06426A3B5D8ULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(uavcan_protocol_enumeration_Begin, UAVCAN_PROTOCOL_ENUMERATION_BEGIN_ID, UAVCAN_PROTOCOL_ENUMERATION_BEGIN_SIGNATURE, UAVCAN_PROTOCOL_ENUMERATION_BEGIN_REQUEST_MAX_SIZE, UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_MAX_SIZE);
#endif
