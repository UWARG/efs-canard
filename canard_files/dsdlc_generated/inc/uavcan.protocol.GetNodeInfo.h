

#pragma once
#include "../inc/uavcan.protocol.GetNodeInfo_req.h"
#include "../inc/uavcan.protocol.GetNodeInfo_res.h"

#define UAVCAN_PROTOCOL_GETNODEINFO_ID 1
#define UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE (0xEE468A8121C46A9EULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(uavcan_protocol_GetNodeInfo, UAVCAN_PROTOCOL_GETNODEINFO_ID, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE, UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_MAX_SIZE, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE);
#endif
