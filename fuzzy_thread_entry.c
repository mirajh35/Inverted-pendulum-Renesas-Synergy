#include <stdlib.h>
#include <math.h>
#include "fuzzy_thread.h"

#define MIN_ERR_ANGLE 900
#define MAX_ERR_ANGLE 1100
#define MIN_DERR_ANGLE -100
#define MAX_DERR_ANGLE 100

#define MIN_ERR_POS -9500
#define MAX_ERR_POS 9500
//#define MIN_DERR_POS -9500
//#define MAX_DERR_POS 9500

#define MF_COUNT 7
#define MF_ITEMS 3
#define DEF_OUT  2

#define ZE 0
#define NS 1
#define NM 2
#define NL 3
#define PS 4
#define PM 5
#define PL 6

#define A 0
#define B 1
#define C 2

/********* Membership functions for error angle **************************/
const float mf_e_angle[MF_COUNT][MF_ITEMS] =
{
//{left,(left-right)/2,right}
  { 967, 33, 1033 },            //ZE
  { 934, 33, 1000 },            //NS
  { 900, 33, 967 },             //NM
  { 900, 33, 934 },             //NL
  { 1000, 33, 1066 },           //PS
  { 1033, 33, 1100 },           //PM
  { 1066, 33, 1100 }            //PL
};
/********* Membership functions for change error angle *******************/
const float mf_de_angle[MF_COUNT][MF_ITEMS] =
{
//{left,(left-right)/2,right}
  { -25, 25, 25 },               //ZE
  { -50, 25, 0 },                //NS
  { -75, 25, -25 },              //NM
  { -100, 25, -50 },             //NL
  { 0, 25, 50 },                 //PS
  { 25, 25, 75 },                //PM
  { 50, 25, 100 }                //PL
};
/********* Membership functions for output angle *************************/
const float mf_out_angle[MF_COUNT] =
{
//ZE,NS,NM,NL,PS,PM,PL
  0,
  -70,
  -85,
  -100,
  70,
  85,
  100 };
/********* Membership functions for error and change error pos ***********/
const float mf_e_pos[MF_COUNT][MF_ITEMS] =
{
//{left,(left-right)/2,right}
  { -2375, 2375, 2375 },         //ZE
  { -4750, 2375, 0 },            //NS
  { -7125, 2375, -2375 },        //NM
  { -9500, 2375, -4750 },        //NL
  { 0, 2375, 4750 },             //PS
  { 2375, 2375, 7125 },          //PM
  { 4750, 2375, 9500 }           //PL
};
/********* Membership functions for output pos ***************************/
const float mf_out_pos[MF_COUNT] =
{
//ZE,NS,NM,NL,PS,PM,PL
  0,
  -16,
  -22,
  -38,
  16,
  22,
  38 };
/********* Rules table ***************************************************/
const int rules_tab[MF_COUNT][MF_COUNT] =
{
{ ZE, NS, NM, NL, PS, PM, PL },
  { NS, NM, NL, NL, ZE, PS, PM },
  { NM, NL, NL, NL, NS, ZE, PS },
  { NL, NL, NL, NL, NM, NS, ZE },
  { PS, ZE, NS, NM, PM, PL, PL },
  { PM, PS, ZE, NS, PL, PL, PL },
  { PL, PM, PS, ZE, PL, PL, PL } };

float mu_in[MF_COUNT][DEF_OUT];
float mu_out_angle[MF_COUNT];
float mu_out_pos[MF_COUNT];

float force_angle = 0;
float force_pos = 0;

float prev_e_angle = 0;
float e_angle = 0;
float de_angle = 0;

float prev_e_pos = 0;
float e_pos = 0;
float de_pos = 0;

static volatile bool time_pendulum = false;
static volatile bool time_cart = false;

float maxd(float x, float y);
float mind(float x, float y);
void fuz(float x, const float f[][MF_ITEMS], float u[][DEF_OUT], int col);
void aggreg(float in[][2], const int rules[][MF_COUNT], float out[]);
float defuz(float out[], const float mu[]);
void Fuz_cart(void);
void Fuz_pendulum(void);

/* Fuzzy Thread entry function */
void fuzzy_thread_entry(void)
{
    g_timer0.p_api->open (g_timer0.p_ctrl, g_timer0.p_cfg);
    g_timer0.p_api->dutyCycleSet (g_timer0.p_ctrl, 0, TIMER_PWM_UNIT_PERCENT, 1);
    g_timer0.p_api->start (g_timer0.p_ctrl);
    g_timer1.p_api->open (g_timer1.p_ctrl, g_timer1.p_cfg);
    g_timer1.p_api->start (g_timer1.p_ctrl);
    g_external_irq0.p_api->open (g_external_irq0.p_ctrl, g_external_irq0.p_cfg);
    g_external_irq0.p_api->enable (g_external_irq0.p_ctrl);
    g_external_irq1.p_api->open (g_external_irq1.p_ctrl, g_external_irq1.p_cfg);
    g_external_irq1.p_api->enable (g_external_irq1.p_ctrl);

    timer_size_t dutycycle = 0;
    float force = 0;
    while (1)
    {
        if (e_angle > MIN_ERR_ANGLE && e_angle < MAX_ERR_ANGLE)
        {
            if (time_pendulum)
                Fuz_pendulum ();
            if (time_cart)
                Fuz_cart ();
            force = force_angle + force_pos;
            if (force < 0)
            {
                if (force < -100)
                    force = -100;
                g_ioport.p_api->pinWrite (IOPORT_PORT_06_PIN_14, IOPORT_LEVEL_HIGH); //motor left
                dutycycle = (timer_size_t) (force * -1);
            }
            else
            {
                if (force > 100)
                    force = 100;
                g_ioport.p_api->pinWrite (IOPORT_PORT_06_PIN_14, IOPORT_LEVEL_LOW);  //motor right
                dutycycle = (timer_size_t) force;
            }
            g_timer0.p_api->dutyCycleSet (g_timer0.p_ctrl, dutycycle % 100, TIMER_PWM_UNIT_PERCENT, 1);
        }
        else
        {
            g_timer0.p_api->dutyCycleSet (g_timer0.p_ctrl, 0, TIMER_PWM_UNIT_PERCENT, 1);
        }
        //tx_thread_sleep (1);
    }
}

void Fuz_cart(void)
{
    e_pos = maxd (mind (e_pos, MAX_ERR_POS), MIN_ERR_POS);
    fuz (e_pos, mf_e_pos, mu_in, 0);
    de_pos = e_pos - prev_e_pos;
    de_pos = maxd (mind (de_pos, MAX_ERR_POS), MIN_ERR_POS);
    prev_e_pos = e_pos;
    fuz (de_pos, mf_e_pos, mu_in, 1);
    time_cart = false;
    aggreg (mu_in, rules_tab, mu_out_pos);
    force_pos = defuz (mu_out_pos, mf_out_pos);
}

void Fuz_pendulum(void)
{
    e_angle = maxd (mind (e_angle, MAX_ERR_ANGLE), MIN_ERR_ANGLE);
    fuz (e_angle, mf_e_angle, mu_in, 0);
    de_angle = e_angle - prev_e_angle;
    de_angle = maxd (mind (de_angle, MAX_DERR_ANGLE), MIN_DERR_ANGLE);
    prev_e_angle = e_angle;
    fuz (de_angle, mf_de_angle, mu_in, 1);
    time_pendulum = false;
    aggreg (mu_in, rules_tab, mu_out_angle);
    force_angle = defuz (mu_out_angle, mf_out_angle);
}

/*********** Callback function - interrupt of cart irc ************************/
void interrupt_pos_irc(external_irq_callback_args_t *p_args)
{
    ioport_level_t state;
    g_ioport.p_api->pinRead (IOPORT_PORT_05_PIN_07, &state);
    if (state)
        e_pos += 0.5f;       //rotate left => +++
    else
        e_pos -= 0.5f;            //rotate right => --
}

/*********** Callback function - interrupt of pendulum irc  *******************/
void interrupt_pen_irc(external_irq_callback_args_t *p_args)
{
    ioport_level_t state;
    g_ioport.p_api->pinRead (IOPORT_PORT_05_PIN_06, &state);
    if (!state)
        e_angle += 0.5f;    //rotate right => ++
    else
        e_angle -= 0.5f;          //rotate left => --
}

/*********** Callback function - timer1 finished ******************************/
void timer1_overflow(timer_callback_args_t *p_args)
{
    time_cart = true;
    time_pendulum = true;
}

float maxd(float x, float y)
{
    return x > y ? x : y;
}

float mind(float x, float y)
{
    return x > y ? y : x;
}

void fuz(float x, const float f[][MF_ITEMS], float u[][DEF_OUT], int col)
{
    for (int i = 0; i < MF_COUNT; i++)
    {
        u[i][col] = maxd (mind ((x - f[i][A]) / f[i][B], (f[i][C] - x) / f[i][B]), 0);
    }
}

void aggreg(float in[][2], const int rules[][MF_COUNT], float out[])
{
    //clear output array
    for (int i = 0; i < MF_COUNT; i++)
    {
        out[i] = 0;
    }

    for (int i = 0; i < MF_COUNT; i++)
    {
        if (fpclassify(in[i][0]) != FP_ZERO)
        {
            for (int j = 0; j < MF_COUNT; j++)
            {
                if (fpclassify(in[j][1]) != FP_ZERO)
                {
                    out[rules[i][j]] = maxd (out[rules[i][j]], mind (in[i][0], in[j][1]));
                }
            }
        }
    }
}

float defuz(float out[], const float mu[])
{
    float num = 0;
    float den = 0;

    for (int i = 0; i < MF_COUNT; i++)
    {
        if (fpclassify(out[i]) != FP_ZERO)
        {
            num += out[i] * mu[i];
            den += out[i];
        }
    }
    return (num / den);
}
