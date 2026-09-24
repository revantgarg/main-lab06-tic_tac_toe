/*
 * tm4c123gh6pm_startup_ccs_gcc.c - startup code for TM4C123GH6PM, GNU toolchain (CCS)
 *
 * Matches the CCS-generated linker script tm4c123gh6pm.lds:
 *   __data_load__ (load address of .data in flash), __data_start__,
 *   __data_end__, __bss_start__, __bss_end__
 * and places the vector table in section ".intvecs" (linked at 0x0).
 *
 * Lab-6 changes vs. previous version:
 *   1. extern comments point to the files that now define the handlers
 *   2. stack aligned to 8 bytes (AAPCS / exception entry requirement)
 *   3. .bss is cleared in assembly - the stack itself lives in .bss, so a
 *      C loop at -O0 would wipe its own loop variables and run away
 *   4. DSB/ISB after enabling the FPU
 *   5. GPIO Port F wired to GPIOPortF_Handler (SW1 start/restart button,
 *      main.c) instead of IntDefaultHandler
 */
#include <stdint.h>

/* ---------------- handlers defined elsewhere ---------------- */
extern int  main(void);
extern void SysTick_Handler(void);      /* main.c         */
extern void GPIOPortC_Handler(void);    /* main.c (keypad)*/
extern void GPIOPortF_Handler(void);    /* main.c (SW1)    */
extern void UART0_Handler(void);        /* UART_Driver.c  */
extern void UART5_Handler(void);        /* UART_Driver.c  */

/* ---------------- local handlers ---------------- */
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

/* ---------------- stack ---------------- */
#define STACK_WORDS 512                 /* 2 KB */
static uint32_t pui32Stack[STACK_WORDS] __attribute__((aligned(8)));

/* ---------------- linker symbols ---------------- */
extern uint32_t __data_load__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

/* ---------------- vector table ---------------- */
__attribute__ ((section(".intvecs"), used))
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((uintptr_t)pui32Stack + sizeof(pui32Stack)),
                                            /* initial stack pointer      */
    ResetISR,                               /* Reset                      */
    NmiSR,                                  /* NMI                        */
    FaultISR,                               /* Hard fault                 */
    IntDefaultHandler,                      /* MPU fault                  */
    IntDefaultHandler,                      /* Bus fault                  */
    IntDefaultHandler,                      /* Usage fault                */
    0, 0, 0, 0,                             /* Reserved                   */
    IntDefaultHandler,                      /* SVCall                     */
    IntDefaultHandler,                      /* Debug monitor              */
    0,                                      /* Reserved                   */
    IntDefaultHandler,                      /* PendSV                     */
    SysTick_Handler,                        /* SysTick                    */

    IntDefaultHandler,                      /*  0 GPIO Port A             */
    IntDefaultHandler,                      /*  1 GPIO Port B             */
    GPIOPortC_Handler,                      /*  2 GPIO Port C  (keypad)   */
    IntDefaultHandler,                      /*  3 GPIO Port D             */
    IntDefaultHandler,                      /*  4 GPIO Port E             */
    UART0_Handler,                          /*  5 UART0  (console)        */
    IntDefaultHandler,                      /*  6 UART1                   */
    IntDefaultHandler,                      /*  7 SSI0                    */
    IntDefaultHandler,                      /*  8 I2C0                    */
    IntDefaultHandler,                      /*  9 PWM0 Fault              */
    IntDefaultHandler,                      /* 10 PWM0 Gen 0              */
    IntDefaultHandler,                      /* 11 PWM0 Gen 1              */
    IntDefaultHandler,                      /* 12 PWM0 Gen 2              */
    IntDefaultHandler,                      /* 13 QEI0                    */
    IntDefaultHandler,                      /* 14 ADC0 Seq 0              */
    IntDefaultHandler,                      /* 15 ADC0 Seq 1              */
    IntDefaultHandler,                      /* 16 ADC0 Seq 2              */
    IntDefaultHandler,                      /* 17 ADC0 Seq 3              */
    IntDefaultHandler,                      /* 18 Watchdog                */
    IntDefaultHandler,                      /* 19 Timer 0A                */
    IntDefaultHandler,                      /* 20 Timer 0B                */
    IntDefaultHandler,                      /* 21 Timer 1A                */
    IntDefaultHandler,                      /* 22 Timer 1B                */
    IntDefaultHandler,                      /* 23 Timer 2A                */
    IntDefaultHandler,                      /* 24 Timer 2B                */
    IntDefaultHandler,                      /* 25 Comparator 0            */
    IntDefaultHandler,                      /* 26 Comparator 1            */
    IntDefaultHandler,                      /* 27 Comparator 2            */
    IntDefaultHandler,                      /* 28 System Control          */
    IntDefaultHandler,                      /* 29 Flash Control           */
    GPIOPortF_Handler,                      /* 30 GPIO Port F  (SW1)      */
    IntDefaultHandler,                      /* 31 GPIO Port G             */
    IntDefaultHandler,                      /* 32 GPIO Port H             */
    IntDefaultHandler,                      /* 33 UART2                   */
    IntDefaultHandler,                      /* 34 SSI1                    */
    IntDefaultHandler,                      /* 35 Timer 3A                */
    IntDefaultHandler,                      /* 36 Timer 3B                */
    IntDefaultHandler,                      /* 37 I2C1                    */
    IntDefaultHandler,                      /* 38 QEI1                    */
    IntDefaultHandler,                      /* 39 CAN0                    */
    IntDefaultHandler,                      /* 40 CAN1                    */
    0, 0,                                   /* 41-42 Reserved             */
    IntDefaultHandler,                      /* 43 Hibernate               */
    IntDefaultHandler,                      /* 44 USB0                    */
    IntDefaultHandler,                      /* 45 PWM0 Gen 3              */
    IntDefaultHandler,                      /* 46 uDMA Software           */
    IntDefaultHandler,                      /* 47 uDMA Error              */
    IntDefaultHandler,                      /* 48 ADC1 Seq 0              */
    IntDefaultHandler,                      /* 49 ADC1 Seq 1              */
    IntDefaultHandler,                      /* 50 ADC1 Seq 2              */
    IntDefaultHandler,                      /* 51 ADC1 Seq 3              */
    0, 0,                                   /* 52-53 Reserved             */
    IntDefaultHandler,                      /* 54 GPIO Port J             */
    IntDefaultHandler,                      /* 55 GPIO Port K             */
    IntDefaultHandler,                      /* 56 GPIO Port L             */
    IntDefaultHandler,                      /* 57 SSI2                    */
    IntDefaultHandler,                      /* 58 SSI3                    */
    IntDefaultHandler,                      /* 59 UART3                   */
    IntDefaultHandler,                      /* 60 UART4                   */
    UART5_Handler,                          /* 61 UART5  (board link)     */
    IntDefaultHandler,                      /* 62 UART6                   */
    IntDefaultHandler,                      /* 63 UART7                   */
    0, 0, 0, 0,                             /* 64-67 Reserved             */
    IntDefaultHandler,                      /* 68 I2C2                    */
    IntDefaultHandler,                      /* 69 I2C3                    */
    IntDefaultHandler,                      /* 70 Timer 4A                */
    IntDefaultHandler,                      /* 71 Timer 4B                */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,           /* 72-81 Reserved             */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,           /* 82-91 Reserved             */
    IntDefaultHandler,                      /* 92 Timer 5A                */
    IntDefaultHandler,                      /* 93 Timer 5B                */
    IntDefaultHandler,                      /* 94 Wide Timer 0A           */
    IntDefaultHandler,                      /* 95 Wide Timer 0B           */
    IntDefaultHandler,                      /* 96 Wide Timer 1A           */
    IntDefaultHandler,                      /* 97 Wide Timer 1B           */
    IntDefaultHandler,                      /* 98 Wide Timer 2A           */
    IntDefaultHandler,                      /* 99 Wide Timer 2B           */
    IntDefaultHandler,                      /* 100 Wide Timer 3A          */
    IntDefaultHandler,                      /* 101 Wide Timer 3B          */
    IntDefaultHandler,                      /* 102 Wide Timer 4A          */
    IntDefaultHandler,                      /* 103 Wide Timer 4B          */
    IntDefaultHandler,                      /* 104 Wide Timer 5A          */
    IntDefaultHandler,                      /* 105 Wide Timer 5B          */
    IntDefaultHandler,                      /* 106 FPU                    */
    0, 0,                                   /* 107-108 Reserved           */
    IntDefaultHandler,                      /* 109 I2C4                   */
    IntDefaultHandler,                      /* 110 I2C5                   */
    IntDefaultHandler,                      /* 111 GPIO Port M            */
    IntDefaultHandler,                      /* 112 GPIO Port N            */
    IntDefaultHandler,                      /* 113 QEI2                   */
    0, 0,                                   /* 114-115 Reserved           */
    IntDefaultHandler,                      /* 116 GPIO Port P0           */
    IntDefaultHandler,                      /* 117 GPIO Port P1           */
    IntDefaultHandler,                      /* 118 GPIO Port P2           */
    IntDefaultHandler,                      /* 119 GPIO Port P3           */
    IntDefaultHandler,                      /* 120 GPIO Port P4           */
    IntDefaultHandler,                      /* 121 GPIO Port P5           */
    IntDefaultHandler,                      /* 122 GPIO Port P6           */
    IntDefaultHandler,                      /* 123 GPIO Port P7           */
    IntDefaultHandler,                      /* 124 GPIO Port Q0           */
    IntDefaultHandler,                      /* 125 GPIO Port Q1           */
    IntDefaultHandler,                      /* 126 GPIO Port Q2           */
    IntDefaultHandler,                      /* 127 GPIO Port Q3           */
    IntDefaultHandler,                      /* 128 GPIO Port Q4           */
    IntDefaultHandler,                      /* 129 GPIO Port Q5           */
    IntDefaultHandler,                      /* 130 GPIO Port Q6           */
    IntDefaultHandler,                      /* 131 GPIO Port Q7           */
    IntDefaultHandler,                      /* 132 GPIO Port R            */
    IntDefaultHandler,                      /* 133 GPIO Port S            */
    IntDefaultHandler,                      /* 134 PWM1 Gen 0             */
    IntDefaultHandler,                      /* 135 PWM1 Gen 1             */
    IntDefaultHandler,                      /* 136 PWM1 Gen 2             */
    IntDefaultHandler,                      /* 137 PWM1 Gen 3             */
    IntDefaultHandler                       /* 138 PWM1 Fault             */
};

_Static_assert(sizeof(g_pfnVectors) / sizeof(g_pfnVectors[0]) == 16 + 139,
               "vector table must have 155 entries");

/* ---------------- reset ---------------- */
void ResetISR(void)
{
    uint32_t *src = &__data_load__;
    uint32_t *dst;

    /* copy initialised data from flash to SRAM */
    for (dst = &__data_start__; dst < &__data_end__; ) {
        *dst++ = *src++;
    }

    /* zero .bss - registers only, because the stack we are running on is
     * itself inside .bss and would be wiped under a C loop's feet        */
    __asm volatile (
        "    ldr     r0, =__bss_start__   \n"
        "    ldr     r1, =__bss_end__     \n"
        "    mov     r2, #0               \n"
        "1:  cmp     r0, r1               \n"
        "    it      lt                   \n"
        "    strlt   r2, [r0], #4         \n"
        "    blt     1b                   \n"
        ::: "r0", "r1", "r2", "cc", "memory");

    /* enable the FPU (CP10/CP11 full access) - needed with -mfloat-abi=hard */
    (*((volatile uint32_t *)0xE000ED88u)) |= (0xFu << 20);
    __asm volatile ("dsb\n\tisb" ::: "memory");

    main();

    for (;;) { }
}

/* ---------------- default handlers ---------------- */
static void NmiSR(void)
{
    for (;;) { }
}

static void FaultISR(void)
{
    for (;;) { }            /* hard fault: attach debugger, check the stack */
}

static void IntDefaultHandler(void)
{
    for (;;) { }            /* unexpected interrupt */
}
