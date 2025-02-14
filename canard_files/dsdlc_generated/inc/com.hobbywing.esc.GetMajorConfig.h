

#pragma once
#include "../inc/com.hobbywing.esc.GetMajorConfig_req.h"
#include "../inc/com.hobbywing.esc.GetMajorConfig_res.h"

#define COM_HOBBYWING_ESC_GETMAJORCONFIG_ID 242
#define COM_HOBBYWING_ESC_GETMAJORCONFIG_SIGNATURE (0x1506774DA3930BFDULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(com_hobbywing_esc_GetMajorConfig, COM_HOBBYWING_ESC_GETMAJORCONFIG_ID, COM_HOBBYWING_ESC_GETMAJORCONFIG_SIGNATURE, COM_HOBBYWING_ESC_GETMAJORCONFIG_REQUEST_MAX_SIZE, COM_HOBBYWING_ESC_GETMAJORCONFIG_RESPONSE_MAX_SIZE);
#endif
