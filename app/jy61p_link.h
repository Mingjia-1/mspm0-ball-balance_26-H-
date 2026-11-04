/**
 * @file jy61p_link.h
 * @brief Application-facing JY61P vehicle-attitude service.
 */
#ifndef APP_JY61P_LINK_H_
#define APP_JY61P_LINK_H_

#include <stdbool.h>

#include "protocol/jy61p_protocol.h"

void Jy61pLink_init(void);
void Jy61pLink_tick1ms(void);
void Jy61pLink_process(void);
bool Jy61pLink_getObservation(Jy61pObservation *observation);

#endif
