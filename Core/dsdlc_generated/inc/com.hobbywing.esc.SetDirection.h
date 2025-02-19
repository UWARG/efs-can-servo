

#pragma once
#include "../inc/com.hobbywing.esc.SetDirection_req.h"
#include "../inc/com.hobbywing.esc.SetDirection_res.h"

#define COM_HOBBYWING_ESC_SETDIRECTION_ID 213
#define COM_HOBBYWING_ESC_SETDIRECTION_SIGNATURE (0x9D793111D262BA68ULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(com_hobbywing_esc_SetDirection, COM_HOBBYWING_ESC_SETDIRECTION_ID, COM_HOBBYWING_ESC_SETDIRECTION_SIGNATURE, COM_HOBBYWING_ESC_SETDIRECTION_REQUEST_MAX_SIZE, COM_HOBBYWING_ESC_SETDIRECTION_RESPONSE_MAX_SIZE);
#endif
