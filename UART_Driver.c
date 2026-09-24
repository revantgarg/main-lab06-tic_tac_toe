/*
 * UART_Driver.c
 *
 * Register-level implementation of UART_Driver.h for the TM4C123GH6PM
 * -- no TivaWare/driverlib. RX is interrupt driven (small per-port ring
 * buffer, filled by each UARTn ISR); TX is a direct, non-blocking write
 * into the 16-byte hardware FIFO, exactly as the header describes ("If
 * the FIFO is full return back with number of bytes that was
 * successfully added"). Generic across UART0-UART7; on this board only
 * port 0 (PA0/PA1, USB virtual COM console) and port 5 (PE4/PE5,
 * board-to-board link) are actually wired up without clashing with the
 * keypad/7-segment pins used elsewhere in the project (see main.c).
 */

#include <stdint.h>
#include "UART_Driver.h"

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define SYSCTL_RCGCGPIO_R   REG32(0x400FE608u)
#define SYSCTL_RCGCUART_R   REG32(0x400FE618u)

#define SYSCTL_RCGCGPIO_PORTA   0x01u
#define SYSCTL_RCGCGPIO_PORTB   0x02u
#define SYSCTL_RCGCGPIO_PORTC   0x04u
#define SYSCTL_RCGCGPIO_PORTD   0x08u
#define SYSCTL_RCGCGPIO_PORTE   0x10u

#define GPIOA_BASE  0x40004000u
#define GPIOB_BASE  0x40005000u
#define GPIOC_BASE  0x40006000u
#define GPIOD_BASE  0x40007000u
#define GPIOE_BASE  0x40024000u

#define GPIO_O_AFSEL  0x420u
#define GPIO_O_DEN    0x51Cu
#define GPIO_O_LOCK   0x520u
#define GPIO_O_CR     0x524u
#define GPIO_O_AMSEL  0x528u
#define GPIO_O_PCTL   0x52Cu
#define GPIO_REG(base, off) REG32((base) + (off))

#define UART0_BASE  0x4000C000u
#define UART1_BASE  0x4000D000u
#define UART2_BASE  0x4000E000u
#define UART3_BASE  0x4000F000u
#define UART4_BASE  0x40010000u
#define UART5_BASE  0x40011000u
#define UART6_BASE  0x40012000u
#define UART7_BASE  0x40013000u

#define UART_O_DR    0x000u
#define UART_O_FR    0x018u
#define UART_O_IBRD  0x024u
#define UART_O_FBRD  0x028u
#define UART_O_LCRH  0x02Cu
#define UART_O_CTL   0x030u
#define UART_O_IM    0x038u
#define UART_O_MIS   0x040u
#define UART_O_ICR   0x044u
#define UART_REG(base, off) REG32((base) + (off))

#define UART_FR_TXFF  0x20u
#define UART_FR_RXFE  0x10u

#define UART_LCRH_SPS   0x80u
#define UART_LCRH_WLEN8 0x60u
#define UART_LCRH_FEN   0x10u
#define UART_LCRH_EPS   0x04u
#define UART_LCRH_PEN   0x02u

#define UART_CTL_RXE    0x0200u
#define UART_CTL_TXE    0x0100u
#define UART_CTL_UARTEN 0x0001u

#define UART_IM_RXIM  0x10u
#define UART_IM_RTIM  0x40u
#define UART_ICR_ALL  0x0FFFu

/* RCGCUART bit for UART port n (0-7) */
#define SYSCTL_RCGCUART_BIT(n) (1u << (n))

#define NVIC_EN0_R  REG32(0xE000E100u)
#define NVIC_EN1_R  REG32(0xE000E104u)

#define IRQ_UART0  5
#define IRQ_UART2  33

/* Enable an IRQ 0-31 -> EN0, 32-63 -> EN1 */
static inline void NVIC_EnableIRQ(int irq)
{
    if (irq < 32) {
        NVIC_EN0_R = (1u << irq);
    } else {
        NVIC_EN1_R = (1u << (irq - 32));
    }
}

/* On reset the TM4C123 runs from the 16 MHz Precision Internal
 * Oscillator (PIOSC) with the PLL bypassed - see main.c for why this
 * project relies on that default instead of configuring the PLL. */
#define SYSCLK_HZ 16000000u

#define UART_NUM_PORTS   8
#define UART_RX_BUF_SIZE 32u

typedef struct {
    uint32_t uart_base;
    uint32_t gpio_base;
    uint32_t gpio_rcgc_bit;
    uint8_t  uart_rcgc_bit;
    uint8_t  rx_pin;
    uint8_t  tx_pin;
    uint8_t  af_num;
    uint8_t  needs_unlock; /* 1 if rx/tx pin is commit-locked (PD7) */
    uint8_t  irq_num;
} uart_port_cfg_t;

/* UART0 PA0/PA1, UART1 PB0/PB1, UART2 PD6/PD7 (PD7 is commit-locked,
 * shared with NMI), UART3 PC6/PC7, UART4 PC4/PC5, UART5 PE4/PE5,
 * UART6 PD4/PD5, UART7 PE0/PE1. All use PCTL alternate-function value 1. */
static const uart_port_cfg_t s_port_cfg[UART_NUM_PORTS] = {
    /*0*/ { UART0_BASE, GPIOA_BASE, SYSCTL_RCGCGPIO_PORTA, 0, 0, 1, 1, 0, IRQ_UART0 },
    /*1*/ { UART1_BASE, GPIOB_BASE, SYSCTL_RCGCGPIO_PORTB, 1, 0, 1, 1, 0, 6 },
    /*2*/ { UART2_BASE, GPIOD_BASE, SYSCTL_RCGCGPIO_PORTD, 2, 6, 7, 1, 1, IRQ_UART2 },
    /*3*/ { UART3_BASE, GPIOC_BASE, SYSCTL_RCGCGPIO_PORTC, 3, 6, 7, 1, 0, 59 },
    /*4*/ { UART4_BASE, GPIOC_BASE, SYSCTL_RCGCGPIO_PORTC, 4, 4, 5, 1, 0, 60 },
    /*5*/ { UART5_BASE, GPIOE_BASE, SYSCTL_RCGCGPIO_PORTE, 5, 4, 5, 1, 0, 61 },
    /*6*/ { UART6_BASE, GPIOD_BASE, SYSCTL_RCGCGPIO_PORTD, 6, 4, 5, 1, 0, 62 },
    /*7*/ { UART7_BASE, GPIOE_BASE, SYSCTL_RCGCGPIO_PORTE, 7, 0, 1, 1, 0, 63 },
};

static volatile unsigned char s_rx_buf[UART_NUM_PORTS][UART_RX_BUF_SIZE];
static volatile uint8_t       s_rx_head[UART_NUM_PORTS];
static volatile uint8_t       s_rx_tail[UART_NUM_PORTS];
static volatile uint8_t       s_port_ready[UART_NUM_PORTS];

int UART_Init(int iPort, int iBaudRate, int iParity_NOEMS)
{
    if (iPort < 0 || iPort >= UART_NUM_PORTS) return -1;
    if (iBaudRate <= 0) return -2;

    const uart_port_cfg_t *cfg = &s_port_cfg[iPort];
    uint8_t rx_mask = (uint8_t)(1u << cfg->rx_pin);
    uint8_t tx_mask = (uint8_t)(1u << cfg->tx_pin);
    uint8_t pin_mask = (uint8_t)(rx_mask | tx_mask);

    s_rx_head[iPort] = 0;
    s_rx_tail[iPort] = 0;
    s_port_ready[iPort] = 0;

    /* Clock the UART module and its GPIO port, then give the clock a
     * few cycles to start before touching the peripheral. */
    SYSCTL_RCGCGPIO_R |= cfg->gpio_rcgc_bit;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_BIT(cfg->uart_rcgc_bit);
    { volatile int delay; for (delay = 0; delay < 32; delay++) { } }

    /* PD7 (UART2 TX) is commit-protected (shared with NMI) - unlock it. */
    if (cfg->needs_unlock) {
        GPIO_REG(cfg->gpio_base, GPIO_O_LOCK) = 0x4C4F434Bu;
        GPIO_REG(cfg->gpio_base, GPIO_O_CR)  |= pin_mask;
    }

    /* Route the pins to the UART peripheral. */
    GPIO_REG(cfg->gpio_base, GPIO_O_AMSEL) &= (uint32_t)~pin_mask;
    GPIO_REG(cfg->gpio_base, GPIO_O_AFSEL) |= pin_mask;
    GPIO_REG(cfg->gpio_base, GPIO_O_DEN)   |= pin_mask;
    {
        uint32_t pctl = GPIO_REG(cfg->gpio_base, GPIO_O_PCTL);
        pctl &= ~((uint32_t)0xFu << (4u * cfg->rx_pin));
        pctl &= ~((uint32_t)0xFu << (4u * cfg->tx_pin));
        pctl |= ((uint32_t)cfg->af_num << (4u * cfg->rx_pin));
        pctl |= ((uint32_t)cfg->af_num << (4u * cfg->tx_pin));
        GPIO_REG(cfg->gpio_base, GPIO_O_PCTL) = pctl;
    }

    UART_REG(cfg->uart_base, UART_O_CTL) &= ~UART_CTL_UARTEN;

    /* Baud-rate divisor: BRD = SYSCLK / (16 * baud), split into an
     * integer part (IBRD) and 6-bit fractional part (FBRD = frac*64),
     * rounded to the nearest 1/64. 64-bit intermediate so this keeps
     * working even if SYSCLK_HZ is raised later. */
    {
        uint64_t num = (uint64_t)SYSCLK_HZ * 64u + (uint64_t)(8u * (uint32_t)iBaudRate);
        uint64_t den = (uint64_t)16u * (uint32_t)iBaudRate;
        uint64_t brd64 = num / den;
        UART_REG(cfg->uart_base, UART_O_IBRD) = (uint32_t)(brd64 >> 6);
        UART_REG(cfg->uart_base, UART_O_FBRD) = (uint32_t)(brd64 & 0x3Fu);
    }

    /* 8 data bits, 1 stop bit, FIFOs enabled, parity as requested. */
    {
        uint32_t lcrh = UART_LCRH_WLEN8 | UART_LCRH_FEN;
        switch (iParity_NOEMS) {
            case UART_PARTITY_NONE:  break;
            case UART_PARTITY_ODD:   lcrh |= UART_LCRH_PEN; break;
            case UART_PARTITY_EVEN:  lcrh |= UART_LCRH_PEN | UART_LCRH_EPS; break;
            case UART_PARTITY_MARK:  lcrh |= UART_LCRH_PEN | UART_LCRH_SPS; break;
            case UART_PARTITY_SPACE: lcrh |= UART_LCRH_PEN | UART_LCRH_SPS | UART_LCRH_EPS; break;
            default: return -3;
        }
        UART_REG(cfg->uart_base, UART_O_LCRH) = lcrh;
    }

    /* Interrupt on RX data and RX timeout only (TX stays polled/direct). */
    UART_REG(cfg->uart_base, UART_O_ICR) = UART_ICR_ALL;
    UART_REG(cfg->uart_base, UART_O_IM)  = UART_IM_RXIM | UART_IM_RTIM;

    UART_REG(cfg->uart_base, UART_O_CTL) |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);

    NVIC_EnableIRQ(cfg->irq_num);

    s_port_ready[iPort] = 1;
    return 0;
}

int UART_Write(int iPort, int iBytes, unsigned char *pcBytes)
{
    if (iPort < 0 || iPort >= UART_NUM_PORTS || !s_port_ready[iPort]) return 0;
    if (pcBytes == 0 || iBytes <= 0) return 0;

    uint32_t base = s_port_cfg[iPort].uart_base;
    int written = 0;
    while (written < iBytes) {
        if (UART_REG(base, UART_O_FR) & UART_FR_TXFF) {
            break; /* hardware FIFO full - non-blocking, stop here */
        }
        UART_REG(base, UART_O_DR) = pcBytes[written];
        written++;
    }
    return written;
}

int UART_Read(int iPort, int iBytes, unsigned char *pcBytes)
{
    if (iPort < 0 || iPort >= UART_NUM_PORTS || !s_port_ready[iPort]) return 0;
    if (pcBytes == 0 || iBytes <= 0) return 0;

    int n = 0;
    while (n < iBytes) {
        if (s_rx_head[iPort] == s_rx_tail[iPort]) break; /* empty */
        pcBytes[n] = s_rx_buf[iPort][s_rx_tail[iPort]];
        s_rx_tail[iPort] = (uint8_t)((s_rx_tail[iPort] + 1u) % UART_RX_BUF_SIZE);
        n++;
    }
    return n;
}

int UART_BytesInRx(int iPort)
{
    if (iPort < 0 || iPort >= UART_NUM_PORTS || !s_port_ready[iPort]) return 0;
    int head = s_rx_head[iPort];
    int tail = s_rx_tail[iPort];
    return (head - tail + (int)UART_RX_BUF_SIZE) % (int)UART_RX_BUF_SIZE;
}

/* Drains the hardware RX FIFO into the software ring buffer for this
 * port. If the ring buffer is full, further bytes are dropped rather
 * than blocking inside the ISR. */
static void UART_ISR_Common(int iPort)
{
    uint32_t base = s_port_cfg[iPort].uart_base;
    uint32_t mis = UART_REG(base, UART_O_MIS);

    if (mis & (UART_IM_RXIM | UART_IM_RTIM)) {
        while (!(UART_REG(base, UART_O_FR) & UART_FR_RXFE)) {
            unsigned char c = (unsigned char)(UART_REG(base, UART_O_DR) & 0xFFu);
            uint8_t next_head = (uint8_t)((s_rx_head[iPort] + 1u) % UART_RX_BUF_SIZE);
            if (next_head != s_rx_tail[iPort]) {
                s_rx_buf[iPort][s_rx_head[iPort]] = c;
                s_rx_head[iPort] = next_head;
            }
        }
    }
    UART_REG(base, UART_O_ICR) = UART_ICR_ALL;
}

/* One thin handler per UART, named to match the vector table your IDE
 * generates. Only UART0_Handler and UART5_Handler ever actually fire on
 * this board, but all eight exist for a genuinely generic driver. */
void UART0_Handler(void) { UART_ISR_Common(0); }
void UART1_Handler(void) { UART_ISR_Common(1); }
void UART2_Handler(void) { UART_ISR_Common(2); }
void UART3_Handler(void) { UART_ISR_Common(3); }
void UART4_Handler(void) { UART_ISR_Common(4); }
void UART5_Handler(void) { UART_ISR_Common(5); }
void UART6_Handler(void) { UART_ISR_Common(6); }
void UART7_Handler(void) { UART_ISR_Common(7); }
