

#pragma once
#include "../inc/uavcan.protocol.param.ExecuteOpcode_req.h"
#include "../inc/uavcan.protocol.param.ExecuteOpcode_res.h"

#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID 10
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE (0x3B131AC5EB69D2CDULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(uavcan_protocol_param_ExecuteOpcode, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_MAX_SIZE, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE);
#endif
