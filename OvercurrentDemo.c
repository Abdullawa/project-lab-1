#include <msp430.h>

/* ==== Pin Map (your wiring) ==== */
#define ENA_BIT   BIT7      /* P2.7 -> ENA (PWM A) */
#define ENB_BIT   BIT6      /* P2.6 -> ENB (PWM B) */
#define IN1_BIT   BIT3      /* P1.3 -> IN1 (Left dir A) */
#define IN2_BIT   BIT0      /* P3.0 -> IN2 (Left dir B) */
#define IN3_BIT   BIT1      /* P3.1 -> IN3 (Right dir A) */
#define IN4_BIT   BIT3      /* P2.3 -> IN4 (Right dir B) */
#define LED_BIT   BIT0      /* P1.0 debug LED */

/* Overcurrent flags (LM339 outputs, active LOW) */
#define OC_B_BIT  BIT6      /* P1.6 = Sense B comparator output */
#define OC_A_BIT  BIT7      /* P1.7 = Sense A comparator output */

/* ==== Clocks / PWM ==== */
#define SMCLK_HZ     1000000UL
#define PWM_FREQ_HZ  20000UL
#define PWM_PERIOD   (SMCLK_HZ / PWM_FREQ_HZ)

/* ==== Globals (unchanged) ==== */
volatile unsigned int dutyA = (PWM_PERIOD * 60) / 100;  /* ~60% duty */
volatile unsigned int dutyB = (PWM_PERIOD * 60) / 100;
volatile unsigned int pwmSub = 0;

/* ===== 20 kHz software PWM (Timer_A0) ===== */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void)
{
    pwmSub++;
    if (pwmSub >= PWM_PERIOD) pwmSub = 0;

    if (pwmSub < dutyA) P2OUT |= ENA_BIT; else P2OUT &= ~ENA_BIT;
    if (pwmSub < dutyB) P2OUT |= ENB_BIT; else P2OUT &= ~ENB_BIT;
}

/* ==== Init functions (unchanged) ==== */
static void clock_init(void)
{
    CSCTL0_H = CSKEY_H;
    CSCTL1   = DCOFSEL_1;  // ~1 MHz
    CSCTL2   = SELS__DCOCLK | SELM__DCOCLK | SELA__VLOCLK;
    CSCTL3   = DIVS__1 | DIVM__1 | DIVA__1;
    CSCTL0_H = 0;
}

static void gpio_init(void)
{
    PM5CTL0 &= ~LOCKLPM5;

    // Motor control pins
    P2DIR |= ENA_BIT | ENB_BIT;  P2OUT &= ~(ENA_BIT | ENB_BIT);
    P1DIR |= IN1_BIT | LED_BIT;  P1OUT &= ~(IN1_BIT | LED_BIT);
    P3DIR |= IN2_BIT | IN3_BIT;  P3OUT &= ~(IN2_BIT | IN3_BIT);
    P2DIR |= IN4_BIT;            P2OUT &= ~IN4_BIT;

    // Overcurrent inputs from LM339 (active LOW)
    P1DIR &= ~(OC_A_BIT | OC_B_BIT); // inputs
    P1REN |=  (OC_A_BIT | OC_B_BIT); // enable pull resistors
    P1OUT |=  (OC_A_BIT | OC_B_BIT); // pull-ups (safety)
}

/* ==== Motor control (unchanged) ==== */
static void set_forward(void)
{
    // Left motor forward
    P1OUT |= IN1_BIT;
    P3OUT &= ~IN2_BIT;

    // Right motor forward
    P3OUT |= IN3_BIT;
    P2OUT &= ~IN4_BIT;
}

static void stop_motors(void)
{
    P1OUT &= ~IN1_BIT;
    P3OUT &= ~IN2_BIT;
    P3OUT &= ~IN3_BIT;
    P2OUT &= ~IN4_BIT;
    dutyA = 0;
    dutyB = 0;
}

/* ================================
   Minimal ADC on SNB -> P9.1 (A9)
   (only changes needed for faster trip)
   ================================ */
#define ADC_TRIP_COUNTS_FAST  280   /* single-sample fast trip */
#define ADC_TRIP_COUNTS       300   /* averaged guard */
#define ADC_SAMPLES           1     /* reduced for faster response */

volatile unsigned dbg_adc = 0;   /* watch in CCS Expressions */

static void adc_init_min(void)
{
    /* Put P9.1 into analog mode for A9 */
    P9SEL0 |= BIT1;
    P9SEL1 |= BIT1;

    /* ADC12_B single-channel on A9, 12-bit, AVCC reference */
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;   /* 16 cycles sample, ADC on */
    ADC12CTL1 = ADC12SHP;                /* sample timer */
    ADC12CTL2 = ADC12RES_2;              /* 12-bit */
    ADC12MCTL0 = ADC12INCH_9;            /* channel A9 */
    ADC12CTL0 |= ADC12ENC;               /* enable conversions */
}

static unsigned adc_read_blocking(void)
{
    ADC12CTL0 |= ADC12SC;                /* start conversion */
    while (ADC12CTL1 & ADC12BUSY);       /* wait */
    return ADC12MEM0;
}

static unsigned adc_read_avg(void)
{
    unsigned acc = 0;
    unsigned i;
    for (i = 0; i < ADC_SAMPLES; i++) acc += adc_read_blocking();
    return acc / ADC_SAMPLES;
}

/* ==== Main (minimal additions only) ==== */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    clock_init();
    gpio_init();

    /* init ADC for SNB on P9.1 (A9) */
    adc_init_min();

    /* Timers init (as you had it) */
    TA0CCR0  = PWM_PERIOD - 1;
    TA0CCTL0 = CCIE;
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;
    __enable_interrupt();

    set_forward();

    /* Small startup grace so motor can begin before we enforce limit */
    __delay_cycles(1000000);  /* ~1s at 1 MHz */

    // Motors run indefinitely, stop if overcurrent
    while (1) {
        /* FAST TRIP: single sample (no averaging/debounce) */
        {
            unsigned v_fast = adc_read_blocking();
            if (v_fast >= ADC_TRIP_COUNTS_FAST) {
                stop_motors();
                P1OUT |= LED_BIT;
                while (1);
            }
        }

        /* Averaged trip (guard) */
        {
            unsigned v = adc_read_avg();
            dbg_adc = v;
            if (v >= ADC_TRIP_COUNTS) {
                stop_motors();
                P1OUT |= LED_BIT;
                while (1);
            }
        }

        /* Original LM339 digital trip (kept unchanged) */
        if (!(P1IN & OC_A_BIT) || !(P1IN & OC_B_BIT)) {
            stop_motors();
            P1OUT |= LED_BIT;
            while (1);
        }
    }
}
