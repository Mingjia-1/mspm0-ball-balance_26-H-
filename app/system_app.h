/** @file system_app.h @brief 1 ms ISR hooks and cooperative scheduler. */
#ifndef APP_SYSTEM_APP_H_
#define APP_SYSTEM_APP_H_

void SystemApp_init(void);
void SystemApp_tick1ms(void);
void SystemApp_process(void);

#endif
