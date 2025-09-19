#include <msp430.h>

/* ==== Pin Map (your wiring) ==== */
#define ENA_BIT   BIT7      /* P2.7 -> ENA (PWM A) */
#define ENB_BIT   BIT6      /* P2.6 -> ENB (PWM B) */
#define IN1_BIT   BIT3      /* P1.3 -> IN1  (Left dir A) */
#define IN2_BIT   BIT0      /* P3.0 -> IN2  (Left dir B) */
#define IN3_BIT   BIT1      /* P3.1 -> IN3  (Right dir A) */
#define IN4_BIT   BIT3      /* P2.3 -> IN4  (Right dir B) */
#define LED_BIT   BIT0      /* P1.0 debug LED */

/* ==== Clocks / PWM ==== */
#define SMCLK_HZ     1000000UL
#define PWM_FREQ_HZ  20000UL
#define PWM_PERIOD   (SMCLK_HZ / PWM_FREQ_HZ)   /* 50 ticks @ 1 MHz */

/* ==== Calibrated times (tune for your floor) ==== */
#define MOVE_1FT_MS   1100   /* ~1 ft */
#define TURN_90_MS     500   /* ~90° (adjust as needed) */

#define FWD_DUTY_PCT    55
#define TURN_DUTY_PCT   60

/* ==== Globals ==== */
volatile unsigned int dutyA = (PWM_PERIOD * FWD_DUTY_PCT) / 100;  /* ENA */
volatile unsigned int dutyB = (PWM_PERIOD * FWD_DUTY_PCT) / 100;  /* ENB */
volatile unsigned int pwmSub = 0;        /* 0..PWM_PERIOD-1 */

volatile unsigned long ms_ticks = 0;     /* 1 ms tick from TB0 */
volatile unsigned char sleep_flag = 0;

/* ==== Prototypes ==== */
static void clock_init(void);
static void gpio_init(void);
static void timers_init(void);

static void set_duty_pct(unsigned pctA, unsigned pctB);
static void soft_start(unsigned pctA, unsigned pctB, unsigned ramp_ms);
static void sleep_ms(unsigned long ms);

static void set_forward(void);
static void set_reverse(void);
static void set_turn_right(void);
static void stop_motors(void);
static void hard_brake_ms(unsigned ms);

static void blink_alive(unsigned n);
static void blink_done(void);     /* NEW: final blink */

/* ===== 20 kHz software PWM (Timer_A0) ===== */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void)
{
    pwmSub++;
    if (pwmSub >= PWM_PERIOD) pwmSub = 0;

    if (pwmSub < dutyA) P2OUT |= ENA_BIT; else P2OUT &= ~ENA_BIT;
    if (pwmSub < dutyB) P2OUT |= ENB_BIT; else P2OUT &= ~ENB_BIT;
}

/* ===== 1 ms tick (Timer_B0) ===== */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TB0_ISR(void)
{
    ms_ticks++;
    sleep_flag = 1;
    __bic_SR_register_on_exit(LPM0_bits);
}

/* ===== Init ===== */
static void clock_init(void)
{
    CSCTL0_H = CSKEY_H;
    CSCTL1   = DCOFSEL_1; /* ~1 MHz (matches timing math) */
    CSCTL2   = SELS__DCOCLK | SELM__DCOCLK | SELA__VLOCLK;
    CSCTL3   = DIVS__1 | DIVM__1 | DIVA__1;
    CSCTL0_H = 0;
}

static void gpio_init(void)
{
    PM5CTL0 &= ~LOCKLPM5;

    P2DIR |= ENA_BIT | ENB_BIT;  P2OUT &= ~(ENA_BIT | ENB_BIT);
    P1DIR |= IN1_BIT | LED_BIT;  P1OUT &= ~(IN1_BIT | LED_BIT);
    P3DIR |= IN2_BIT | IN3_BIT;  P3OUT &= ~(IN2_BIT | IN3_BIT);
    P2DIR |= IN4_BIT;            P2OUT &= ~IN4_BIT;
}

static void timers_init(void)
{
    /* TA0: 20 kHz ISR */
    TA0CCR0  = PWM_PERIOD - 1;                 /* 49 @ 1 MHz */
    TA0CCTL0 = CCIE;
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;

    /* TB0: 1 kHz ms tick */
    TB0CCR0  = (SMCLK_HZ / 1000U) - 1;         /* 999 */
    TB0CCTL0 = CCIE;
    TB0CTL   = TBSSEL__SMCLK | MC__UP | TBCLR;

    __enable_interrupt();
}

/* ===== Helpers ===== */
static void set_duty_pct(unsigned pctA, unsigned pctB)
{
    if (pctA > 100) pctA = 100;
    if (pctB > 100) pctB = 100;
    dutyA = (PWM_PERIOD * pctA) / 100;
    dutyB = (PWM_PERIOD * pctB) / 100;
}

static void soft_start(unsigned pctA, unsigned pctB, unsigned ramp_ms)
{
    unsigned i, steps = 10;
    unsigned a = 0, b = 0;
    if (pctA > 100) pctA = 100;
    if (pctB > 100) pctB = 100;

    for (i = 0; i <= steps; i++) {
        a = (pctA * i) / steps;
        b = (pctB * i) / steps;
        set_duty_pct(a, b);
        sleep_ms(ramp_ms / steps);
    }
}

static void sleep_ms(unsigned long ms)
{
    unsigned long target = ms_ticks + ms;
    while ((long)(target - ms_ticks) > 0) {
        sleep_flag = 0;
        __bis_SR_register(LPM0_bits | GIE);
    }
}

/* Direction sets (swap if sides are reversed) */
static void set_forward(void)
{
    /* Left motor: IN1=1, IN2=0 ; Right motor: IN3=1, IN4=0 */
    P1OUT |= IN1_BIT;    P3OUT &= ~IN2_BIT;   /* left forward */
    P3OUT |= IN3_BIT;    P2OUT &= ~IN4_BIT;   /* right forward */
}

static void set_reverse(void)
{
    P1OUT &= ~IN1_BIT;   P3OUT |= IN2_BIT;
    P3OUT &= ~IN3_BIT;   P2OUT |= IN4_BIT;
}

static void set_turn_right(void)
{
    /* left fwd, right rev */
    P1OUT |= IN1_BIT;    P3OUT &= ~IN2_BIT;   /* left forward */
    P3OUT &= ~IN3_BIT;   P2OUT |= IN4_BIT;    /* right reverse */
}
static void set_turn_left(void)
{
    /* left rev, right fwd */
    P1OUT &= ~IN1_BIT;   P3OUT |= IN2_BIT;    /* left reverse */
    P3OUT |= IN3_BIT;    P2OUT &= ~IN4_BIT;   /* right forward */
}


static void stop_motors(void)
{
    P1OUT &= ~IN1_BIT;
    P3OUT &= ~IN2_BIT;
    P3OUT &= ~IN3_BIT;
    P2OUT &= ~IN4_BIT;
    set_duty_pct(0, 0);
}


/* Active brake for crisp stopping (keeps EN high briefly) */
static void hard_brake_ms(unsigned ms)
{
    /* both inputs HIGH */
    P1OUT |= IN1_BIT;  P3OUT |= IN2_BIT;
    P3OUT |= IN3_BIT;  P2OUT |= IN4_BIT;
    set_duty_pct(100, 100);      /* short the windings via H-bridge */
    sleep_ms(ms);
    stop_motors();               /* then disable */
}

static void blink_alive(unsigned n)
{
    unsigned i;
    for (i = 0; i < n; i++) {
        P1OUT |= LED_BIT;  sleep_ms(120);
        P1OUT &= ~LED_BIT; sleep_ms(180);
    }
    sleep_ms(250);
}

/* Final confirmation blink: quick 2–1 pattern */
static void blink_done(void)
{
    P1OUT |= LED_BIT; sleep_ms(120);
    P1OUT &= ~LED_BIT; sleep_ms(120);
    P1OUT |= LED_BIT; sleep_ms(120);
    P1OUT &= ~LED_BIT; sleep_ms(300);
    P1OUT |= LED_BIT; sleep_ms(250);
    P1OUT &= ~LED_BIT;
}

/* ===== Main: Forward 1 ft -> 90° turn -> stop (one-shot) ===== */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    clock_init();
    gpio_init();
    timers_init();

    blink_alive(3);

    /* --- Forward ~1 ft --- */
    set_forward();
    soft_start(FWD_DUTY_PCT, FWD_DUTY_PCT, 200);
    sleep_ms(MOVE_1FT_MS);

    /* Brake + pause */
    hard_brake_ms(150);
    sleep_ms(250);

    /* --- Turn ~90° right --- */
    set_turn_right();
    soft_start(TURN_DUTY_PCT, TURN_DUTY_PCT, 300);
    sleep_ms(TURN_90_MS);

    /* Final brake + stop */
    hard_brake_ms(120);
    stop_motors();

    /* Done indicator */
    blink_done();

    /* Stay idle */
    while (1) {
        __bis_SR_register(LPM0_bits | GIE);
    }
}