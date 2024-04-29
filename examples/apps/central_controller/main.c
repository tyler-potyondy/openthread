/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>

#include <openthread-system.h>
#include <openthread/dataset_ftd.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/message.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/udp.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/tasklet.h>
#include <openthread/platform/logging.h>

#include "openthread-system.h"
#include "cli/cli_config.h"
#include "common/code_utils.hpp"
#include "utils/uart.h"

#include <utils/code_utils.h>
#include "lib/platform/reset_util.h"


#define UDP_PORT 1212

bool reply = false;

static const char UDP_DEST_ADDR[] = "ff02::1";
static char UDP_PAYLOAD[]   = "Hello OpenThread World!";

static void initUdp(otInstance *aInstance);
static void sendUdp(otInstance *aInstance);

static otUdpSocket sUdpSocket;
extern uint8_t otTemperatureInput(otIp6Address ipv6, uint8_t temperature);

void handleUdpReceive(void *aContext, otMessage *aMessage, 
                      const otMessageInfo *aMessageInfo);

/**
 * Initializes the CLI app.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
extern void otAppCliInit(otInstance *aInstance);
static void setNetworkConfiguration(otInstance *aInstance);


#if OPENTHREAD_CONFIG_HEAP_EXTERNAL_ENABLE
OT_TOOL_WEAK void *otPlatCAlloc(size_t aNum, size_t aSize) { return calloc(aNum, aSize); }

OT_TOOL_WEAK void otPlatFree(void *aPtr) { free(aPtr); }
#endif

void otTaskletsSignalPending(otInstance *aInstance) { OT_UNUSED_VARIABLE(aInstance); }


int main(int argc, char *argv[])
{
    otInstance *instance;

    OT_SETUP_RESET_JUMP(argv);

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    size_t   otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer       = NULL;
#endif

pseudo_reset:

    otSysInit(argc, argv);

#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);

    // Initialize OpenThread with the buffer
    instance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    instance = otInstanceInitSingle();
#endif
    assert(instance);
    otAppCliInit(instance);

    setNetworkConfiguration(instance);

    // uint8_t buf[7] = {'h', 'e', 'l', 'l', 'o','\r','\n'};
    
    
    // otPlatUartSend(buf,7);
    // otPlatUartFlush();

	otIp6SetEnabled(instance, true);

    initUdp(instance);

	otThreadSetEnabled(instance, true);

    while (!otSysPseudoResetWasRequested())
    {
        otTaskletsProcess(instance);
        otSysProcessDrivers(instance);
    }

    otInstanceFinalize(instance);
#if OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    free(otInstanceBuffer);
#endif

    goto pseudo_reset;

    return 0;
}


void setNetworkConfiguration(otInstance *aInstance)
{
	otOperationalDataset aDataset;
    otOperationalDatasetTlvs sDatasetTlvs;


    otDatasetCreateNewNetwork(aInstance, &aDataset);
    otDatasetConvertToTlvs(&aDataset, &sDatasetTlvs);

	memset(&aDataset, 0, sizeof(otOperationalDataset));

	/* Set Channel to 26 */
	aDataset.mChannel = 26;
	aDataset.mComponents.mIsChannelPresent = true;

	/* Set Pan ID to abcd */
	aDataset.mPanId = (otPanId)0xabcd;
	aDataset.mComponents.mIsPanIdPresent = true;

	/* Set network key to 00112233445566778899aabbccddeeff */
	uint8_t key[OT_NETWORK_KEY_SIZE] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		          0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
	memcpy(aDataset.mNetworkKey.m8, key, sizeof(aDataset.mNetworkKey));
	aDataset.mComponents.mIsNetworkKeyPresent = true;

    otDatasetUpdateTlvs(&aDataset, &sDatasetTlvs);
    
    otError error = otDatasetSetActiveTlvs(aInstance, &sDatasetTlvs);

	assert(error == 0);
}

/**
 * Initialize UDP socket
 */
void initUdp(otInstance *aInstance)
{
    otSockAddr  listenSockAddr;

    memset(&sUdpSocket, 0, sizeof(sUdpSocket));
    memset(&listenSockAddr, 0, sizeof(listenSockAddr));

    listenSockAddr.mPort    = UDP_PORT;

    otUdpOpen(aInstance, &sUdpSocket, handleUdpReceive, aInstance);
    otUdpBind(aInstance, &sUdpSocket, &listenSockAddr, OT_NETIF_THREAD);
}



/**
 * Send a UDP datagram
 */
void sendUdp(otInstance *aInstance)
{
    otError       error = OT_ERROR_NONE;
    otMessage *   message;
    otMessageInfo messageInfo;
    otIp6Address  destinationAddr;

    memset(&messageInfo, 0, sizeof(messageInfo));

    otIp6AddressFromString(UDP_DEST_ADDR, &destinationAddr);
    messageInfo.mPeerAddr    = destinationAddr;
    messageInfo.mPeerPort    = UDP_PORT;

    message = otUdpNewMessage(aInstance, NULL);
    otEXPECT_ACTION(message != NULL, error = OT_ERROR_NO_BUFS);

    error = otMessageAppend(message, UDP_PAYLOAD, sizeof(UDP_PAYLOAD));
    otEXPECT(error == OT_ERROR_NONE);

    error = otUdpSend(aInstance, &sUdpSocket, message, &messageInfo);

 exit:
    if (error != OT_ERROR_NONE && message != NULL)
    {
        otMessageFree(message);
    }
}

/**
 * Function to handle UDP datagrams received on the listening socket
 */
void handleUdpReceive(void *aContext, otMessage *aMessage,
                      const otMessageInfo *aMessageInfo)
{
    OT_UNUSED_VARIABLE(aContext);
    OT_UNUSED_VARIABLE(aMessage);
    OT_UNUSED_VARIABLE(aMessageInfo);
      char buf[20];
  int length;

  length      = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);

  uint8_t temperature = buf[0];


    // const uint8_t val[1] = 'a'};
    
    // otPlatUartEnable();
    // // otPlatUartSend((const uint8_t*) &buf,1);
    // otPlatUartSend(val, 1);
    // otPlatUartFlush();
    // otPlatUartDisable();
    
    // based on the sender's address, update the stored temperature 
    // for that child
    uint8_t avg_temp = otTemperatureInput(aMessageInfo->mPeerAddr, temperature);

    UDP_PAYLOAD[0] = avg_temp;

    sendUdp((otInstance *)aContext);
}

