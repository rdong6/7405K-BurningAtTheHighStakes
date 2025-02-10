#pragma once

extern "C" {
void vTaskGetRunTimeStats(char* pcWriteBuffer);
}

// macros to enter / exit critical section
#define enter_critical() asm("cpsid i"); asm("dsb"); asm("isb")
#define exit_critical() asm("cpsie i"); asm("dsb"); asm("isb")