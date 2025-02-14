/*
 * canard_stm32_driver.h
 *
 *  Created on: Jul 9, 2024
 *      Author: ronik
 */

#ifndef INC_CANARD_STM32_DRIVER_H_
#define INC_CANARD_STM32_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

int16_t canardSTM32Recieve(FDCAN_HandleTypeDef *hfdcan, uint32_t RxLocation, CanardCANFrame *const rx_frame);
int16_t canardSTM32Transmit(FDCAN_HandleTypeDef *hfdcan, const CanardCANFrame* const tx_frame);
void getUniqueID(uint8_t id[16]);

#ifdef __cplusplus
}
#endif

#endif /* INC_CANARD_STM32_DRIVER_H_ */
