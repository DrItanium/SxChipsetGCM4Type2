//
// Created by jwscoggins on 8/28/21.
//

#ifndef HITAGIMON_INTERRUPTS_H
#define HITAGIMON_INTERRUPTS_H
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*InterruptFunction)(void);
InterruptFunction getNMIFunction();
InterruptFunction getISR0Function();
void setNMIFunction(InterruptFunction func);
void setISR0Function(InterruptFunction func);
#ifdef __cplusplus
}
#endif
#endif //HITAGIMON_INTERRUPTS_H
