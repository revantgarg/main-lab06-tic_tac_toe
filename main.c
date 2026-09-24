/*
 * main.c - Lab-6 Tic-Tac-Toe over UART (EduARM4 + TM4C123 LaunchPad)
 *
 * Two boards, each on an EduARM4 trainer, play tic-tac-toe against each
 * other. Each board reads its own player's moves from the first 3 rows
 * x 3 columns of the on-board 4x4 keypad, shows a per-move countdown on
 * the 7-segment display, uses the on-board RGB LED as a turn indicator,
 * and exchanges moves with the other board over UART5. Game status
 * ("Invalid move", "Game won by Player 1/2", "Game Draw") is printed to
 * a PC terminal over UART0 (the LaunchPad's USB virtual COM port).
 *
 * Everything is written directly against the TM4C123GH6PM register set
 * -- no TivaWare/driverlib anywhere -- including the UART driver, which
 * was handed out as the header-only interface in UART_Driver.h and is
 * implemented from scratch below (interrupt-driven RX, ring buffer,
 * non-blocking TX).
 *
 * Split into UART_Driver.h (exactly as given), UART_Driver.c (its
 * implementation), and this file (everything else). Sections below are
 * in dependency order (registers -> config -> game rules -> each
 * driver -> the app logic that wires them together), so nothing needs
 * a forward declaration.
 *
 * ---------------------------------------------------------------------
 * PIN MAP (from the EduARM4 + LaunchPad reference pages)
 *   Console UART (UART0)     PA0 (RX) / PA1 (TX)  - USB virtual COM port
 *   Board-link UART (UART5)  PE4 (RX) / PE5 (TX)  - wire between boards
 *   Keypad rows              PE0-PE3 (open-drain outputs, idle = low)
 *   Keypad columns           PC4-PC7 (inputs, pull-up, falling-edge IRQ)
 *   7-seg segments a..g,dp   PB0-PB7 (common cathode, active-HIGH)
 *   7-seg digit select       PA4 = tens, PA5 = ones (active-HIGH)
 *   RGB LED                  PF1 = red, PF3 = green
 *   Start/restart button     PF4 (SW1, active-low)
 * PC0-PC3 (JTAG/SWD) are never touched. SW2/PF0 is not used at all.
 *
 * BOARD-LINK PROTOCOL (UART5, 9600 8N1 by default)
 *   'S'          - "start/restart": sender becomes Player 1 (X) and it
 *                  is now their move. Receiver becomes Player 2 (O).
 *   'M' <cell>   - sender just placed their symbol in <cell> (0-8).
 * There is no separate "you win"/"timeout"/"game over" message: both
 * boards mirror the identical board state move-for-move and the
 * identical per-move countdown, so every board reaches the same
 * conclusion (win/draw/timeout) independently and at the same time.
 *
 * ASSUMPTIONS TO DOUBLE-CHECK ON YOUR PHYSICAL BOARDS
 *   - Clock: left at the reset-default 16 MHz internal oscillator
 *     (PIOSC), no PLL -- see SYSCLK_HZ below.
 *   - 7-seg digit order: PA4 assumed tens, PA5 assumed ones - swap
 *     TENS_BIT/ONES_BIT below if they show up reversed.
 *   - 7-seg digit-select polarity: assumed active-HIGH.
 *   - Move timer: 15 s/move (APP_MOVE_TIME_SEC below).
 * This has been syntax-checked and the game-rules logic (Game_Reset /
 * Game_IsValidMove / Game_ApplyMove / Game_CheckResult) unit-tested on
 * a PC, but not compiled for ARM or run on real hardware.
 * ---------------------------------------------------------------------
 */

#include <stdint.h>
#include "UART_Driver.h"

/* Port identifiers for iPort, 0-7 map 1:1 onto the TM4C123's UART0-UART7. */
#define UART_PORT_CONSOLE  0   /* UART0, PA0/PA1  - USB virtual COM (console) */
#define UART_PORT_LINK     5   /* UART5, PE4/PE5  - board-to-board link       */

/* ======================================================================
 * SECTION 1: TM4C123GH6PM register access (no TivaWare/driverlib)
 * ==================================================================== */
#define REG32(addr) (*(volatile uint32_t *)(addr))

#define SYSCTL_RCGCGPIO_R   REG32(0x400FE608u)

#define SYSCTL_RCGCGPIO_PORTA   0x01u
#define SYSCTL_RCGCGPIO_PORTB   0x02u
#define SYSCTL_RCGCGPIO_PORTC   0x04u
#define SYSCTL_RCGCGPIO_PORTD   0x08u
#define SYSCTL_RCGCGPIO_PORTE   0x10u
#define SYSCTL_RCGCGPIO_PORTF   0x20u

#define GPIOA_BASE  0x40004000u
#define GPIOB_BASE  0x40005000u
#define GPIOC_BASE  0x40006000u
#define GPIOD_BASE  0x40007000u
#define GPIOE_BASE  0x40024000u
#define GPIOF_BASE  0x40025000u

/* Common offsets inside every GPIO port block */
#define GPIO_O_DIR    0x400u
#define GPIO_O_IS     0x404u
#define GPIO_O_IBE    0x408u
#define GPIO_O_IEV    0x40Cu
#define GPIO_O_IM     0x410u
#define GPIO_O_MIS    0x418u
#define GPIO_O_ICR    0x41Cu
#define GPIO_O_AFSEL  0x420u
#define GPIO_O_ODR    0x50Cu
#define GPIO_O_PUR    0x510u
#define GPIO_O_PDR    0x514u
#define GPIO_O_DEN    0x51Cu
#define GPIO_O_LOCK   0x520u
#define GPIO_O_CR     0x524u
#define GPIO_O_AMSEL  0x528u
#define GPIO_O_PCTL   0x52Cu

/* Bit-masked DATA access: only the bits set in `mask` are read/written.
 * Hardware feature of the TM4C GPIO address decode (address bits [9:2]
 * act as the byte mask), so e.g. GPIO_DATA(GPIOC_BASE,0xF0) touches
 * PC7-PC4 only and can never disturb PC3-PC0 (JTAG/SWD). */
#define GPIO_DATA(base, mask) REG32((base) + ((uint32_t)(mask) << 2))
#define GPIO_REG(base, off)   REG32((base) + (off))

#define NVIC_EN0_R  REG32(0xE000E100u)
#define NVIC_EN1_R  REG32(0xE000E104u)

/* IRQ numbers used in this project (from the TM4C123 vector table) */
#define IRQ_GPIOC  2
#define IRQ_GPIOF  30

/* Enable an IRQ 0-31 -> EN0, 32-63 -> EN1 */
static inline void NVIC_EnableIRQ(int irq)
{
    if (irq < 32) {
        NVIC_EN0_R = (1u << irq);
    } else {
        NVIC_EN1_R = (1u << (irq - 32));
    }
}

#define NVIC_ST_CTRL_R    REG32(0xE000E010u)
#define NVIC_ST_RELOAD_R  REG32(0xE000E014u)
#define NVIC_ST_CURRENT_R REG32(0xE000E018u)

#define NVIC_ST_CTRL_ENABLE   0x01u
#define NVIC_ST_CTRL_INTEN    0x02u
#define NVIC_ST_CTRL_CLK_SRC  0x04u

/* On reset the TM4C123 runs from the 16 MHz Precision Internal
 * Oscillator (PIOSC) with the PLL bypassed - no clock-init code is
 * needed to get exactly 16 MHz, which avoids an entire class of "PLL
 * never locks" bring-up bugs. UART baud-rate and SysTick timing are
 * both derived from this constant; update it if you add your own
 * PLL init to run faster. */
#define SYSCLK_HZ 16000000u

/* ======================================================================
 * SECTION 2: shared app tunables
 * ==================================================================== */
#define APP_TICK_PERIOD_MS     2u   /* SysTick period                        */
#define APP_MOVE_TIME_SEC      15u  /* per-move countdown                    */
#define APP_KEYPAD_DEBOUNCE_MS 250u
#define APP_BUTTON_DEBOUNCE_MS 300u
#define APP_CONSOLE_BAUD  115200
#define APP_LINK_BAUD     9600

/* ======================================================================
 * SECTION 3: game rules engine (no hardware dependency at all)
 *
 * Board layout (matches the keypad's first 3 rows x 3 columns):
 *   0 1 2
 *   3 4 5
 *   6 7 8
 * ==================================================================== */
#define GAME_NUM_CELLS 9

typedef enum { CELL_EMPTY = 0, CELL_X = 1, CELL_O = 2 } cell_t;
typedef enum { GAME_ONGOING = 0, GAME_WON, GAME_DRAW } game_status_t;

typedef struct {
    cell_t cells[GAME_NUM_CELLS];
    int    move_count;
} game_board_t;

static const int s_win_lines[8][3] = {
    {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, /* rows       */
    {0, 3, 6}, {1, 4, 7}, {2, 5, 8}, /* columns    */
    {0, 4, 8}, {2, 4, 6}            /* diagonals  */
};

static void Game_Reset(game_board_t *board)
{
    int i;
    for (i = 0; i < GAME_NUM_CELLS; i++) {
        board->cells[i] = CELL_EMPTY;
    }
    board->move_count = 0;
}

/* A move is valid if the cell index is 0-8 and that cell is still empty. */
static int Game_IsValidMove(const game_board_t *board, int cell)
{
    if (cell < 0 || cell >= GAME_NUM_CELLS) {
        return 0;
    }
    return board->cells[cell] == CELL_EMPTY;
}

/* Caller must have already checked Game_IsValidMove(). */
static void Game_ApplyMove(game_board_t *board, int cell, cell_t player)
{
    board->cells[cell] = player;
    board->move_count++;
}

/* Checks the board for a win/draw after the move just applied by
 * `last_player`. If GAME_WON is returned, *out_winner is set to the
 * winning symbol. */
static game_status_t Game_CheckResult(const game_board_t *board, cell_t last_player, cell_t *out_winner)
{
    int i;
    for (i = 0; i < 8; i++) {
        int a = s_win_lines[i][0];
        int b = s_win_lines[i][1];
        int c = s_win_lines[i][2];
        if (board->cells[a] == last_player &&
            board->cells[b] == last_player &&
            board->cells[c] == last_player) {
            if (out_winner != 0) {
                *out_winner = last_player;
            }
            return GAME_WON;
        }
    }
    if (board->move_count >= GAME_NUM_CELLS) {
        return GAME_DRAW;
    }
    return GAME_ONGOING;
}

/* ======================================================================
 * SECTION 4: keypad driver
 *
 * Rows PE0-PE3: open-drain outputs, idle state = all four driven low.
 * Columns PC4-PC7: inputs with internal pull-ups, falling-edge interrupt.
 *
 * With every row held low, the columns normally read all 1s (pulled
 * up). The instant any key is pressed, its column gets shorted to its
 * (low) row and falls to 0 -- that edge fires GPIOC's interrupt. The
 * ISR then runs the same row-by-row identification your board's keypad
 * guide describes (drive one row low at a time, see which column
 * responds), restores the idle all-rows-low state, and hands the
 * resolved cell to the main loop through a small pending-event flag.
 *
 * cell = row * 3 + col; only the first 3 rows/columns map onto the
 * game's 3x3 grid, the keypad's 4th row/column is reported as
 * KEYPAD_NO_KEY (outside the grid).
 * ==================================================================== */
#define KEYPAD_NO_KEY (-1)
#define ROW_MASK 0x0Fu /* PE3-PE0 */
#define COL_MASK 0xF0u /* PC7-PC4 */

static volatile uint32_t g_keypad_ms;
static volatile uint32_t s_last_event_ms;
static volatile int      s_have_pending;
static volatile int      s_pending_cell;

static void Keypad_Init(void)
{
    SYSCTL_RCGCGPIO_R |= (SYSCTL_RCGCGPIO_PORTC | SYSCTL_RCGCGPIO_PORTE);
    { volatile int d; for (d = 0; d < 32; d++) { } }

    /* Rows PE0-PE3: GPIO output, open-drain. */
    GPIO_REG(GPIOE_BASE, GPIO_O_AFSEL) &= (uint32_t)~ROW_MASK;
    GPIO_REG(GPIOE_BASE, GPIO_O_AMSEL) &= (uint32_t)~ROW_MASK;
    GPIO_REG(GPIOE_BASE, GPIO_O_ODR)  |= ROW_MASK;
    GPIO_REG(GPIOE_BASE, GPIO_O_DIR)  |= ROW_MASK;
    GPIO_REG(GPIOE_BASE, GPIO_O_DEN)  |= ROW_MASK;
    GPIO_DATA(GPIOE_BASE, ROW_MASK) = 0x00u; /* idle: all rows driven low */

    /* Columns PC4-PC7: GPIO input, pull-up, falling-edge interrupt.
     * PC0-PC3 (JTAG/SWD) are never touched - every access below is
     * masked to bits [7:4] only. */
    GPIO_REG(GPIOC_BASE, GPIO_O_AFSEL) &= (uint32_t)~COL_MASK;
    GPIO_REG(GPIOC_BASE, GPIO_O_AMSEL) &= (uint32_t)~COL_MASK;
    GPIO_REG(GPIOC_BASE, GPIO_O_DIR)   &= (uint32_t)~COL_MASK;
    GPIO_REG(GPIOC_BASE, GPIO_O_PDR)   &= (uint32_t)~COL_MASK;
    GPIO_REG(GPIOC_BASE, GPIO_O_PUR)   |= COL_MASK;
    GPIO_REG(GPIOC_BASE, GPIO_O_DEN)   |= COL_MASK;

    GPIO_REG(GPIOC_BASE, GPIO_O_IM)  &= (uint32_t)~COL_MASK;
    GPIO_REG(GPIOC_BASE, GPIO_O_IS)  &= (uint32_t)~COL_MASK; /* edge sensitive */
    GPIO_REG(GPIOC_BASE, GPIO_O_IBE) &= (uint32_t)~COL_MASK; /* single edge    */
    GPIO_REG(GPIOC_BASE, GPIO_O_IEV) &= (uint32_t)~COL_MASK; /* falling edge   */
    GPIO_REG(GPIOC_BASE, GPIO_O_ICR)  = COL_MASK;
    GPIO_REG(GPIOC_BASE, GPIO_O_IM)  |= COL_MASK;

    NVIC_EnableIRQ(IRQ_GPIOC);

    g_keypad_ms = 0;
    s_last_event_ms = 0;
    s_have_pending = 0;
    s_pending_cell = KEYPAD_NO_KEY;
}

/* Returns 1 and writes the resolved cell (0-8) into *cell if a
 * debounced key press is waiting; returns 0 otherwise. Non-blocking. */
static int Keypad_GetKey(int *cell)
{
    if (!s_have_pending) return 0;
    s_have_pending = 0;
    if (cell != 0) *cell = s_pending_cell;
    return 1;
}

/* Discards any pending key event - call this when a turn starts so a
 * stray press made during the opponent's turn isn't replayed. */
static void Keypad_Flush(void)
{
    s_have_pending = 0;
}

void GPIOPortC_Handler(void)
{
    uint32_t mis = GPIO_REG(GPIOC_BASE, GPIO_O_MIS) & COL_MASK;
    if (mis == 0) return;

    int found_row = -1;
    int found_col = -1;
    int r;

    for (r = 0; r < 4 && found_row < 0; r++) {
        /* Drive row r low, release (Hi-Z) the other three. */
        GPIO_DATA(GPIOE_BASE, ROW_MASK) = (uint32_t)(~(1u << r) & ROW_MASK);
        { volatile int d; for (d = 0; d < 8; d++) { } } /* let the line settle */

        uint32_t cols = GPIO_DATA(GPIOC_BASE, COL_MASK);
        if (cols != COL_MASK) {
            int c;
            for (c = 0; c < 4; c++) {
                if (!(cols & (uint32_t)(1u << (4 + c)))) {
                    found_row = r;
                    found_col = c;
                    break;
                }
            }
        }
    }

    /* Re-arm: back to idle with every row driven low. */
    GPIO_DATA(GPIOE_BASE, ROW_MASK) = 0x00u;
    GPIO_REG(GPIOC_BASE, GPIO_O_ICR) = COL_MASK;

    if (found_row >= 0) {
        uint32_t now = g_keypad_ms;
        int too_soon = (now - s_last_event_ms) < APP_KEYPAD_DEBOUNCE_MS;
        if (s_have_pending || too_soon) {
            return; /* previous press not yet consumed, or bounce noise */
        }
        s_pending_cell = (found_row <= 2 && found_col <= 2)
                              ? (found_row * 3 + found_col)
                              : KEYPAD_NO_KEY; /* 4th row/col: outside the 3x3 grid */
        s_have_pending = 1;
        s_last_event_ms = now;
    }
}

/* ======================================================================
 * SECTION 5: 7-segment display driver
 *
 * PB0-PB7 = segments a,b,c,d,e,f,g,dp (common cathode, active-HIGH,
 * exact patterns from the board's own 7-segment guide). PA4/PA5 = the
 * tens/ones digit-select lines (active-HIGH); PA6/PA7 unused, and
 * PA0-PA3 (incl. the UART0 console pins) are never touched.
 * ==================================================================== */
#define SEG_MASK 0xFFu
#define TENS_BIT 4u /* PA4 = Digit-A -> tens place */
#define ONES_BIT 5u /* PA5 = Digit-B -> ones place */
#define DIGIT_SEL_MASK ((1u << TENS_BIT) | (1u << ONES_BIT))

/* Common-cathode segment patterns for 0-9, bit0=a ... bit6=g, bit7=dp. */
static const uint8_t s_digit_pattern[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

static volatile uint8_t s_pattern[2]; /* [0]=tens, [1]=ones, 0 = blank */
static volatile int     s_active_digit;

static void SevenSeg_Init(void)
{
    SYSCTL_RCGCGPIO_R |= (SYSCTL_RCGCGPIO_PORTA | SYSCTL_RCGCGPIO_PORTB);
    { volatile int d; for (d = 0; d < 32; d++) { } }

    GPIO_REG(GPIOB_BASE, GPIO_O_AFSEL) &= (uint32_t)~SEG_MASK;
    GPIO_REG(GPIOB_BASE, GPIO_O_AMSEL) &= (uint32_t)~SEG_MASK;
    GPIO_REG(GPIOB_BASE, GPIO_O_DIR)  |= SEG_MASK;
    GPIO_REG(GPIOB_BASE, GPIO_O_DEN)  |= SEG_MASK;
    GPIO_DATA(GPIOB_BASE, SEG_MASK) = 0x00u;

    GPIO_REG(GPIOA_BASE, GPIO_O_AFSEL) &= (uint32_t)~DIGIT_SEL_MASK;
    GPIO_REG(GPIOA_BASE, GPIO_O_AMSEL) &= (uint32_t)~DIGIT_SEL_MASK;
    GPIO_REG(GPIOA_BASE, GPIO_O_DIR)  |= DIGIT_SEL_MASK;
    GPIO_REG(GPIOA_BASE, GPIO_O_DEN)  |= DIGIT_SEL_MASK;
    GPIO_DATA(GPIOA_BASE, DIGIT_SEL_MASK) = 0x00u;

    s_pattern[0] = 0;
    s_pattern[1] = 0;
    s_active_digit = 0;
}

/* Turn both digits off. */
static void SevenSeg_Blank(void)
{
    s_pattern[0] = 0;
    s_pattern[1] = 0;
}

/* Show a two-digit countdown, 0-99; out of range blanks the display. */
static void SevenSeg_SetValue(int value)
{
    if (value < 0 || value > 99) {
        SevenSeg_Blank();
        return;
    }
    int tens = value / 10;
    int ones = value % 10;
    /* Blank a leading zero in the tens place so e.g. "7" doesn't show as "07". */
    s_pattern[1] = (tens == 0) ? 0 : s_digit_pattern[tens];
    s_pattern[0] = s_digit_pattern[ones];
}

/* Call once per SysTick tick; lights one digit per call, alternating. */
static void SevenSeg_Refresh(void)
{
    GPIO_DATA(GPIOA_BASE, DIGIT_SEL_MASK) = 0x00u; /* blank while switching */

    uint8_t pattern = s_pattern[s_active_digit];
    GPIO_DATA(GPIOB_BASE, SEG_MASK) = pattern;

    uint32_t sel_bit = (s_active_digit == 0) ? (1u << TENS_BIT) : (1u << ONES_BIT);
    if (pattern != 0) {
        GPIO_DATA(GPIOA_BASE, DIGIT_SEL_MASK) = sel_bit;
    }
    s_active_digit = 1 - s_active_digit;
}

/* ======================================================================
 * SECTION 6: LED driver - onboard RGB LED (PF1 = red, PF3 = green) as
 * the turn indicator: blinking green = your move, solid red = waiting.
 * ==================================================================== */
typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_YOUR_TURN, /* blinking green */
    LED_STATE_WAIT_TURN  /* solid red      */
} led_state_t;

#define RED_BIT   1u /* PF1 */
#define GREEN_BIT 3u /* PF3 */
#define LED_MASK  ((1u << RED_BIT) | (1u << GREEN_BIT))
#define BLINK_PERIOD_TICKS (250u / APP_TICK_PERIOD_MS) /* ~250 ms half-period */

static volatile led_state_t s_led_state;
static volatile uint32_t    s_led_tick_count;
static volatile int         s_green_on;

static void LED_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_PORTF;
    { volatile int d; for (d = 0; d < 32; d++) { } }

    /* PF1/PF3 only - PF0 (locked/NMI) and PF2/PF4 are left untouched. */
    GPIO_REG(GPIOF_BASE, GPIO_O_AFSEL) &= (uint32_t)~LED_MASK;
    GPIO_REG(GPIOF_BASE, GPIO_O_AMSEL) &= (uint32_t)~LED_MASK;
    GPIO_REG(GPIOF_BASE, GPIO_O_DIR)  |= LED_MASK;
    GPIO_REG(GPIOF_BASE, GPIO_O_DEN)  |= LED_MASK;
    GPIO_DATA(GPIOF_BASE, LED_MASK) = 0x00u;

    s_led_state = LED_STATE_OFF;
    s_led_tick_count = 0;
    s_green_on = 0;
}

static void LED_SetState(led_state_t state)
{
    s_led_state = state;
    s_led_tick_count = 0;
    switch (state) {
        case LED_STATE_YOUR_TURN:
            s_green_on = 1;
            GPIO_DATA(GPIOF_BASE, LED_MASK) = (1u << GREEN_BIT);
            break;
        case LED_STATE_WAIT_TURN:
            GPIO_DATA(GPIOF_BASE, LED_MASK) = (1u << RED_BIT);
            break;
        case LED_STATE_OFF:
        default:
            GPIO_DATA(GPIOF_BASE, LED_MASK) = 0x00u;
            break;
    }
}

/* Call once per SysTick tick; drives the green blink timing. */
static void LED_Tick(void)
{
    if (s_led_state != LED_STATE_YOUR_TURN) return;
    s_led_tick_count++;
    if (s_led_tick_count >= BLINK_PERIOD_TICKS) {
        s_led_tick_count = 0;
        s_green_on = !s_green_on;
        GPIO_DATA(GPIOF_BASE, (1u << GREEN_BIT)) = s_green_on ? (1u << GREEN_BIT) : 0u;
    }
}

/* ======================================================================
 * SECTION 7: SysTick timer
 *
 * Ticks every APP_TICK_PERIOD_MS and does three things: refreshes one
 * 7-seg digit, gives the keypad/LED drivers a millisecond time base,
 * and accumulates a "1 second elapsed" pulse the main loop consumes to
 * count down the current player's remaining move time.
 * ==================================================================== */
static volatile uint32_t s_ms_in_second;
static volatile int      s_second_flag;
static volatile uint32_t s_ms_total;

static void SysTick_Init(void)
{
    uint32_t reload = (SYSCLK_HZ / 1000u) * APP_TICK_PERIOD_MS - 1u;

    NVIC_ST_CTRL_R    = 0;
    NVIC_ST_RELOAD_R  = reload;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R    = NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;

    s_ms_in_second = 0;
    s_second_flag = 0;
    s_ms_total = 0;
}

/* Free-running millisecond counter, for debouncing the SW1 button. */
static uint32_t SysTick_GetMs(void)
{
    return s_ms_total;
}

/* Returns 1 the first time it is called after a full second has
 * elapsed since the previous "consumed" second, 0 otherwise. */
static int SysTick_ConsumeSecondTick(void)
{
    if (s_second_flag) {
        s_second_flag = 0;
        return 1;
    }
    return 0;
}

void SysTick_Handler(void)
{
    SevenSeg_Refresh();
    g_keypad_ms += APP_TICK_PERIOD_MS;
    LED_Tick();

    s_ms_total += APP_TICK_PERIOD_MS;
    s_ms_in_second += APP_TICK_PERIOD_MS;
    if (s_ms_in_second >= 1000u) {
        s_ms_in_second -= 1000u;
        s_second_flag = 1;
    }
}

/* ======================================================================
 * SECTION 8: SW1 (PF4) start/restart button
 * ==================================================================== */
#define SW1_BIT (1u << 4)

static volatile int      s_sw1_pending;
static volatile uint32_t s_sw1_last_ms;

static void Button_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_PORTF;
    { volatile int d; for (d = 0; d < 32; d++) { } }

    GPIO_REG(GPIOF_BASE, GPIO_O_AFSEL) &= (uint32_t)~SW1_BIT;
    GPIO_REG(GPIOF_BASE, GPIO_O_AMSEL) &= (uint32_t)~SW1_BIT;
    GPIO_REG(GPIOF_BASE, GPIO_O_DIR)   &= (uint32_t)~SW1_BIT;
    GPIO_REG(GPIOF_BASE, GPIO_O_PUR)   |= SW1_BIT; /* SW1 is active-low */
    GPIO_REG(GPIOF_BASE, GPIO_O_DEN)   |= SW1_BIT;

    GPIO_REG(GPIOF_BASE, GPIO_O_IM)  &= (uint32_t)~SW1_BIT;
    GPIO_REG(GPIOF_BASE, GPIO_O_IS)  &= (uint32_t)~SW1_BIT;
    GPIO_REG(GPIOF_BASE, GPIO_O_IBE) &= (uint32_t)~SW1_BIT;
    GPIO_REG(GPIOF_BASE, GPIO_O_IEV) &= (uint32_t)~SW1_BIT; /* falling edge */
    GPIO_REG(GPIOF_BASE, GPIO_O_ICR)  = SW1_BIT;
    GPIO_REG(GPIOF_BASE, GPIO_O_IM)  |= SW1_BIT;

    NVIC_EnableIRQ(IRQ_GPIOF);

    s_sw1_pending = 0;
    s_sw1_last_ms = 0;
}

void GPIOPortF_Handler(void)
{
    uint32_t mis = GPIO_REG(GPIOF_BASE, GPIO_O_MIS) & SW1_BIT;
    if (mis) {
        GPIO_REG(GPIOF_BASE, GPIO_O_ICR) = SW1_BIT;
        uint32_t now = SysTick_GetMs();
        if ((now - s_sw1_last_ms) >= APP_BUTTON_DEBOUNCE_MS) {
            s_sw1_pending = 1;
            s_sw1_last_ms = now;
        }
    }
}

static int Button_ConsumePress(void)
{
    if (s_sw1_pending) {
        s_sw1_pending = 0;
        return 1;
    }
    return 0;
}

/* ======================================================================
 * SECTION 9: tiny console helpers (no libc - fully freestanding)
 * ==================================================================== */
static void console_print(const char *s)
{
    int len = 0;
    while (s[len] != '\0') len++;
    const unsigned char *p = (const unsigned char *)s;
    int sent = 0;
    while (sent < len) {
        int n = UART_Write(UART_PORT_CONSOLE, len - sent, (unsigned char *)(p + sent));
        sent += n;
        if (n == 0) {
            volatile int spin;
            for (spin = 0; spin < 100; spin++) { } /* FIFO momentarily full */
        }
    }
}

static void print_move_line(const char *who, int cell)
{
    char buf[40];
    int i = 0;
    const char *p;
    for (p = who; *p; p++) buf[i++] = *p;
    for (p = " played cell "; *p; p++) buf[i++] = *p;
    buf[i++] = (char)('0' + cell);
    buf[i++] = '\r';
    buf[i++] = '\n';
    buf[i] = '\0';
    console_print(buf);
}

static void print_board(const game_board_t *board)
{
    char buf[64];
    int idx = 0, r, c;
    for (r = 0; r < 3; r++) {
        for (c = 0; c < 3; c++) {
            cell_t v = board->cells[r * 3 + c];
            buf[idx++] = (v == CELL_X) ? 'X' : (v == CELL_O) ? 'O' : '.';
            if (c < 2) buf[idx++] = ' ';
        }
        buf[idx++] = '\r';
        buf[idx++] = '\n';
    }
    buf[idx] = '\0';
    console_print(buf);
}

static void announce_winner(cell_t winner)
{
    console_print((winner == CELL_X) ? "Game won by Player 1\r\n" : "Game won by Player 2\r\n");
}

/* ======================================================================
 * SECTION 10: game state machine + board-link protocol + main()
 * ==================================================================== */
typedef enum { APP_IDLE, APP_PLAYING, APP_GAME_OVER } app_state_t;
typedef enum { TURN_ME, TURN_OPP } turn_owner_t;

static game_board_t   s_board;
static cell_t         s_my_symbol;
static cell_t         s_opp_symbol;
static app_state_t    s_state;
static turn_owner_t   s_turn;
static int            s_remaining_sec;

static void begin_my_turn(void)
{
    Keypad_Flush(); /* discard any stray press made during the wait */
    s_turn = TURN_ME;
    s_remaining_sec = (int)APP_MOVE_TIME_SEC;
    SevenSeg_SetValue(s_remaining_sec);
    LED_SetState(LED_STATE_YOUR_TURN);
    console_print("Your move.\r\n");
}

static void begin_opp_turn(void)
{
    s_turn = TURN_OPP;
    s_remaining_sec = (int)APP_MOVE_TIME_SEC;
    SevenSeg_SetValue(s_remaining_sec);
    LED_SetState(LED_STATE_WAIT_TURN);
    console_print("Waiting for opponent...\r\n");
}

static void start_new_game_as_player1(void)
{
    s_my_symbol = CELL_X;
    s_opp_symbol = CELL_O;
    Game_Reset(&s_board);
    {
        unsigned char msg = 'S';
        UART_Write(UART_PORT_LINK, 1, &msg);
    }
    console_print("\r\n--- New game --- You are Player 1 (X).\r\n");
    s_state = APP_PLAYING;
    begin_my_turn();
}

/* Called from the link-protocol poller when a peer 'S' arrives. */
static void on_remote_start(void)
{
    s_my_symbol = CELL_O;
    s_opp_symbol = CELL_X;
    Game_Reset(&s_board);
    console_print("\r\n--- New game --- Opponent started. You are Player 2 (O).\r\n");
    s_state = APP_PLAYING;
    begin_opp_turn();
}

/* Called from the link-protocol poller when a peer 'M' <cell> arrives. */
static void on_remote_move(int cell)
{
    cell_t winner;
    game_status_t result;

    if (s_state != APP_PLAYING || s_turn != TURN_OPP) return; /* stray - ignore */
    if (!Game_IsValidMove(&s_board, cell)) return;             /* corrupted - ignore */

    Game_ApplyMove(&s_board, cell, s_opp_symbol);
    print_move_line("Opponent", cell);
    print_board(&s_board);

    result = Game_CheckResult(&s_board, s_opp_symbol, &winner);
    if (result == GAME_WON) {
        announce_winner(winner);
        s_state = APP_GAME_OVER;
        LED_SetState(LED_STATE_OFF);
        SevenSeg_Blank();
    } else if (result == GAME_DRAW) {
        console_print("Game Draw\r\n");
        s_state = APP_GAME_OVER;
        LED_SetState(LED_STATE_OFF);
        SevenSeg_Blank();
    } else {
        begin_my_turn();
    }
}

/* Drains one link-protocol message per call, tolerating the 'M'
 * message's payload byte arriving on a later call than its type byte. */
static void poll_link(void)
{
    static int have_type = 0;
    static unsigned char msg_type;
    unsigned char b;

    if (!have_type) {
        if (UART_Read(UART_PORT_LINK, 1, &b) != 1) return;
        msg_type = b;
        have_type = 1;
    }

    if (msg_type == 'S') {
        on_remote_start();
        have_type = 0;
    } else if (msg_type == 'M') {
        if (UART_Read(UART_PORT_LINK, 1, &b) == 1) {
            on_remote_move((int)b);
            have_type = 0;
        }
        /* else: payload not here yet - keep waiting on the next poll */
    } else {
        have_type = 0; /* unknown byte - drop and resynchronise */
    }
}

static void handle_local_keypad(void)
{
    int cell;
    cell_t winner;
    game_status_t result;

    if (s_state != APP_PLAYING || s_turn != TURN_ME) return;
    if (!Keypad_GetKey(&cell)) return;
    if (cell == KEYPAD_NO_KEY || !Game_IsValidMove(&s_board, cell)) {
        console_print("Invalid move\r\n");
        return;
    }

    Game_ApplyMove(&s_board, cell, s_my_symbol);
    print_move_line("You", cell);
    print_board(&s_board);
    {
        unsigned char msg[2];
        msg[0] = 'M';
        msg[1] = (unsigned char)cell;
        UART_Write(UART_PORT_LINK, 2, msg);
    }

    result = Game_CheckResult(&s_board, s_my_symbol, &winner);
    if (result == GAME_WON) {
        announce_winner(winner);
        s_state = APP_GAME_OVER;
        LED_SetState(LED_STATE_OFF);
        SevenSeg_Blank();
    } else if (result == GAME_DRAW) {
        console_print("Game Draw\r\n");
        s_state = APP_GAME_OVER;
        LED_SetState(LED_STATE_OFF);
        SevenSeg_Blank();
    } else {
        begin_opp_turn();
    }
}

static void handle_timer(int second_elapsed)
{
    if (s_state != APP_PLAYING || !second_elapsed) return;
    if (s_remaining_sec > 0) s_remaining_sec--;
    if (s_remaining_sec <= 0) {
        cell_t winner = (s_turn == TURN_ME) ? s_opp_symbol : s_my_symbol;
        console_print("TIME OUT!\r\n");
        announce_winner(winner);
        s_state = APP_GAME_OVER;
        LED_SetState(LED_STATE_OFF);
        SevenSeg_Blank();
    } else {
        SevenSeg_SetValue(s_remaining_sec);
    }
}

int main(void)
{
    UART_Init(UART_PORT_CONSOLE, APP_CONSOLE_BAUD, UART_PARTITY_NONE);
    UART_Init(UART_PORT_LINK, APP_LINK_BAUD, UART_PARTITY_NONE);
    Keypad_Init();
    SevenSeg_Init();
    LED_Init();
    Button_Init();
    SysTick_Init();

    Game_Reset(&s_board);
    s_state = APP_IDLE;
    s_my_symbol = CELL_EMPTY;
    s_opp_symbol = CELL_EMPTY;

    console_print("\r\n=== Lab-6 Tic-Tac-Toe ===\r\n");
    console_print("Press SW1 to start as Player 1 (X). Waiting for you or your opponent...\r\n");

    for (;;) {
        int second_elapsed = SysTick_ConsumeSecondTick();

        poll_link();

        if (s_state == APP_IDLE || s_state == APP_GAME_OVER) {
            if (Button_ConsumePress()) {
                start_new_game_as_player1();
            }
        } else if (s_state == APP_PLAYING) {
            handle_local_keypad();
            handle_timer(second_elapsed);
        }
    }
}
