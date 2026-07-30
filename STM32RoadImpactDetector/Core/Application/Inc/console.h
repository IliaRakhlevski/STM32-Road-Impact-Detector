/**
 * @file console.h
 * @brief Thread-safe console interface.
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdbool.h>

bool Console_Init(void);
int Console_Printf(const char *format, ...);

#endif /* CONSOLE_H_ */
