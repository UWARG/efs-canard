

#pragma once
#include "../inc/uavcan.protocol.RestartNode_req.h"
#include "../inc/uavcan.protocol.RestartNode_res.h"

#define UAVCAN_PROTOCOL_RESTARTNODE_ID 5
#define UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE (0x569E05394A3017F0ULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(uavcan_protocol_RestartNode, UAVCAN_PROTOCOL_RESTARTNODE_ID, UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE, UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAX_SIZE, UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE);
#endif
