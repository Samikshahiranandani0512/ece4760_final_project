/**
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *
 * Protothreads v1.1.1
 * Serial console on GPIO 0 and 1 for debugging
 *
 * DAC :
 * GPIO 5 (pin 7) Chip select
 * GPIO 6 (pin 9) SCK/spi0_sclk
 * GPIO 7 (pin 10) MOSI/spi0_tx
 * 3.3v (pin 36) -> VCC on DAC
 * GND (pin 3)  -> GND on DAC
 * 
 * ==========
 * Support routines:
 * -- ADC setup

 * ==========
 * Core1:
 *
 * -- synthesis ISR triggered by a timer alarm
 * ---- runs at 40KHz (25 uSec interval) for audio synthesis.
 * ---- Computes DDS to get modulating wave, then computes output wave
 * ---- Pushes output to an SPI channel
 */

#include "vga16_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/spi.h"

// ==========================================
// === hardware and protothreads globals
// ==========================================
#include "hardware/sync.h"
#include "song.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "string.h"
// protothreads header
#include "pt_cornell_rp2040_v1_1_1.h"

float serial_value;
char serial_value_input[32];
bool printParams = true;
bool modifiedParams = true;

static float conversion_factor = 3.3f / (1 << 12);
#define ADC_PIN_1 28
#define ADC_PIN_2 27
#define SONG1_BUTTON 16
#define SONG2_BUTTON 17
#define SONG3_BUTTON 18
#define SONG4_BUTTON 19
#define SONG5_BUTTON 20

#define INSTRUMENT1_BUTTON 21
#define INSTRUMENT2_BUTTON 2
#define INSTRUMENT3_BUTTON 3
#define INSTRUMENT4_BUTTON 15

#define NUM_SONGS 5
#define NUM_INSTRUMENTS 4


int song_buttons[NUM_SONGS] = {SONG1_BUTTON, SONG2_BUTTON, SONG3_BUTTON, SONG4_BUTTON, SONG5_BUTTON}; 
int instrument_buttons[NUM_INSTRUMENTS] = {INSTRUMENT1_BUTTON, INSTRUMENT2_BUTTON, INSTRUMENT3_BUTTON, INSTRUMENT4_BUTTON};

int song_speeds[NUM_SONGS] = {4, 2, 2, 4, 1};

note_t* chosen_song;
long chosen_song_len;

void ADC_setup(void) {
    adc_init();
    adc_gpio_init(28);
    adc_gpio_init(27);
    adc_select_input(2);
}

// ==================================================
// === set up a menu scheme 
// ==================================================
// ALL items store and update using float values
// int values are copied from the float
struct menu_item {
    // displayed string
    char item_name[15];
    //1=float 0=int ;  
    int item_data_type;
    // 1=log 0=linear
    int item_inc_type;
    // increment delta if linear, ratio if log
    float item_increment;
    // the current value, min, max 
    // al updates are performed on float, then copied to int
    int item_int_value;
    float item_float_value;
    float item_float_min;
    float item_float_max;
};

// ======
// build the menu array of item parameters to tune values for different 
// instruments
struct menu_item menu[16];
#define menu_length 11

// ===== change value with serial
// direction is 1 for increase,  -1 for decrease
void change_value_serial(int index, float value) {
    menu[index].item_float_value = value;
    // check min/max
    if (menu[index].item_float_value > menu[index].item_float_max)
        menu[index].item_float_value = menu[index].item_float_max;
    if (menu[index].item_float_value < menu[index].item_float_min)
        menu[index].item_float_value = menu[index].item_float_min;
    // update integer to match
    menu[index].item_int_value = (int)menu[index].item_float_value;
}

// ==========================================
// === fixed point s19x12 for DDS
// ==========================================
// s19x12 fixed point macros == for DDS
typedef signed int fix;
#define mul(a,b) ((fix)(((( signed long long )(a))*(( signed long long )(b)))>>12)) //multiply two fixed 16:16
#define float_to_fix(a) ((fix)((a)*4096.0)) // 2^12
#define fix_to_float(a) ((float)(a)/4096.0)
#define fix_to_int(a)    ((int)((a)>>12))
#define int_to_fix(a)    ((fix)((a)<<12))
#define div(a,b) ((fix)((((signed long long)(a)<<12)/(b)))) 
#define absfix(a) abs(a)
typedef signed short s1x14;
#define muls1x14(a,b) ((s1x14)((((int)(a))*((int)(b)))>>14)) 
#define float_to_s1x14(a) ((s1x14)((a)*16384.0)) // 2^14
#define s1x14_to_float(a) ((float)(a)/16384.0)
#define abss1x14(a) abs(a) 
#define divs1x14(a,b) ((s1x14)((((signed int)(a)<<14)/(b)))) 
// shift 12 bits into 14 bits so full scale dds is about 0.25
#define dds_to_s1x14(a) ((s1x14)((a)>>14))

// ==========================================
// === set up SPI DAC
// ==========================================
// All SPI DAC setup was gotten from HUnter Adams
// https://vanhunteradams.com/Pico/TimerIRQ/SPI_DDS.html
// DAC parameters
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

#define MUX_SEL_A 12
#define MUX_SEL_B 11
#define MUX_SEL_C 10
#define MUX_SEL_D 13
static int mux_select = 0;
static float voltage1 = 3;
static float voltage2 = 3;

// data for the spi port
uint16_t DAC_data;

// ==========================================
// === set up DDS and timer ISR
// ==========================================
// 1/Fs in microseconds ~ 27.7kHz
volatile int alarm_period = 36;

#define NUM_KEYS 50
#define NUM_PHYSICAL_KEYS 16
#define VOLTAGE_CUTOFF 1.2
#define BUFFER_COUNT 8

// DDS variables 
unsigned int mod_inc[NUM_KEYS], main_inc[NUM_KEYS];
unsigned int current_mod_inc[NUM_KEYS], current_main_inc[NUM_KEYS];
unsigned int mod_accum[NUM_KEYS], main_accum[NUM_KEYS];

// amplitude paramters
fix max_mod_depth;
fix current_mod_depth[NUM_KEYS];
// waveform amplities -- must fit in +/-11 bits for DAC
fix max_amp = float_to_fix(1000.0);
fix current_amp[NUM_KEYS];

// timing in seconds
static volatile fix attack_time, mod_attack_time;
static volatile fix decay_time, mod_decay_time, recip_decay_time;
static volatile fix sustain_time, mod_sustain_time;
fix onefix = int_to_fix(1);
// internal timing in samples
fix note_time[NUM_KEYS];
static volatile fix attack_inc, decay_inc, mod_attack_inc, mod_decay_inc;
// sine waves
fix sine_table[256];
fix mod_wave[NUM_KEYS], main_wave[NUM_KEYS];
// inputs
float Fs, Fmod;

int base_note = 36;
float notes[NUM_KEYS];
int note_start[NUM_KEYS], play_note[NUM_KEYS], add_delay[NUM_KEYS], buffer[BUFFER_COUNT];
bool pressed[NUM_KEYS], prev_pressed[NUM_KEYS], play_song[NUM_SONGS];
int linear_dk = 0;
int octave_num;

// ==========================================
// === set up timer ISR  used in this pgm
// ==========================================
// === timer alarm ========================

#define ALARM_NUM 1
#define ALARM_IRQ TIMER_IRQ_1
// ISR interval will be 10 uSec
//
// the actual ISR
void compute_sample(void);
//
static void alarm_irq(void) {
    // mark ISR entry
    gpio_put(2, 1);
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    // arm the next interrupt
    // Write the lower 32 bits of the target time to the alarm to arm it
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + alarm_period;
    //
    compute_sample();

    // mark ISR exit
    gpio_put(2, 0);
}

// set up the timer alarm ISR
static void alarm_in_us(uint32_t delay_us) {
    // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Set irq handler for alarm irq
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm irq
    irq_set_enabled(ALARM_IRQ, true);
    // Enable interrupt in block and at processor
    // Alarm is only 32 bits 
    uint64_t target = timer_hw->timerawl + delay_us;
    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[ALARM_NUM] = (uint32_t)target;
}

void add_note(int note);
void remove_note(int note);
void print_notes(void);

void set_chosen_song(int song_num) {
    switch (song_num) {
        case 0: 
            chosen_song = song_data1;
            chosen_song_len = song_len1;
            break; 
        case 1: 
            chosen_song = song_data2; 
            chosen_song_len = song_len2;
            break;
        case 2: 
            chosen_song = song_data3; 
            chosen_song_len = song_len3;
            break;
        case 3: 
            chosen_song = song_data4; 
            chosen_song_len = song_len4;
            break;
        case 4: 
            chosen_song = song_data5; 
            chosen_song_len = song_len5;
            break;
        default: 
            chosen_song = song_data1; 
            chosen_song_len = song_len1; 
    }
}

static PT_THREAD(protothread_playsong(struct pt* pt))
{
    PT_BEGIN(pt);

        static int key1;
        static int key2;
        static int hold_time;
        static int delay_tick = 1000; 


        static int i, j;

        while(1) {

            for (j = 0; j < NUM_SONGS; j++) {
                if (play_song[j]) { 
                    set_chosen_song(j);
                    for(i = 0; i < chosen_song_len; i++) {

                        hold_time = chosen_song[i].hold_time;
                        if (hold_time > 0) {
                            PT_YIELD_usec(hold_time * delay_tick * song_speeds[j]);
                        }

                        key1 = chosen_song[i].notes_press - base_note;
                        if (key1 >= 0 && key1 < NUM_KEYS) {
                            prev_pressed[key1] = pressed[key1] = true;
                            current_main_inc[key1] = main_inc[key1];
                            current_mod_inc[key1] = mod_inc[key1];
                            note_start[key1] = true;
                            add_note(key1);
                            //print_notes();
                            //printf("Playing %d\n",key1);
                        }
                        key2 = chosen_song[i].notes_release - base_note;
                        if (key2 >= 0 && key2 < NUM_KEYS) {
                            pressed[key2] = prev_pressed[key2] = false;
                        }

                        if (!play_song[j]) {
                            // release all keys that could be pressed 
                            for (int m = 0; m < BUFFER_COUNT; m++) {
                                pressed[buffer[m]] = prev_pressed[buffer[m]] = false;
                            }
                            break;
                        }
                 
                    }
                }
                PT_YIELD_usec(10000);
            }

        }
        

    PT_END(pt);
} // play song thread

static PT_THREAD(protothread_buttonpress(struct pt* pt))
{
    PT_BEGIN(pt);
    while(1) {
        for (int i = 0; i < NUM_SONGS; i++) {
            if(!gpio_get(song_buttons[i])) {
                play_song[i] = !play_song[i];
                printf("Button %d pressed - value now %d", i, play_song[i]);
                sleep_ms(250);
            } 
        }
        //harp
        if (!gpio_get(instrument_buttons[0])) {
            menu[0].item_float_value = 3;// "Octave # ") ;
            menu[1].item_float_value = .0;// "Attack main ") ;
            menu[2].item_float_value = .0;// "Sustain main ") ;
            menu[3].item_float_value = .5;// "Decay main ") ;  
            menu[4].item_float_value = 2;// "Fmod/Fmain ") ;
            menu[5].item_float_value = 2;// "FM depth max ") ;
            menu[6].item_float_value = .0;// "Attack FM ") ;
            menu[7].item_float_value = .0;// "Sustain FM ") ;
            menu[8].item_float_value = .4;// "Decay FM ") ;
            menu[9].item_float_value = 0;// "Lin=1/Quad DK ") ;
            menu[10].item_float_value = 1;//  "Run ") ;
        }
        else if (!gpio_get(instrument_buttons[1])) {   
            menu[0].item_float_value = 1 ;   //"Octave # ") ;
            menu[1].item_float_value = 0.001 ;  //"Attack main ") ;
            menu[2].item_float_value = 0 ;  //"Sustain main ") ;
            menu[3].item_float_value = 0.99 ;   //"Decay main ") ;  
            menu[4].item_float_value = 1.6 ;   //"Fmod/Fmain ") ;
            menu[5].item_float_value = 1.5 ;  //"FM depth max ") ;
            menu[6].item_float_value = 0.001 ;  //"Attack FM ") ;
            menu[7].item_float_value = 0 ;   //"Sustain FM ") ;
            menu[8].item_float_value = 0.90 ;   //"Decay FM ") ;
            menu[9].item_float_value = 1 ;  //"Lin=1/Quad DK ") ;
            menu[10].item_float_value = 1 ;  // "Run ") ;
        }
        else if (!gpio_get(instrument_buttons[2])) { 

            menu[0].item_float_value = 3;// "Octave # ") ;
            menu[1].item_float_value = .01;// "Attack main ") ;
            menu[2].item_float_value = .0;// "Sustain main ") ;
            menu[3].item_float_value = 3;// "Decay main ") ;  
            menu[4].item_float_value = 3;// "Fmod/Fmain ") ;
            menu[5].item_float_value = 0;// "FM depth max ") ;
            menu[6].item_float_value = .01;// "Attack FM ") ;
            menu[7].item_float_value = 0;// "Sustain FM ") ;
            menu[8].item_float_value = 3;// "Decay FM ") ;
            menu[9].item_float_value = 0;// "Lin=1/Quad DK ") ;
            menu[10].item_float_value = 1;//  "Run ") ;
        }
        //piano
        else if (!gpio_get(instrument_buttons[3])) {
            menu[0].item_float_value = 3;// "Octave # ") ;
            menu[1].item_float_value = .01;// "Attack main ") ;
            menu[2].item_float_value = .3;// "Sustain main ") ;
            menu[3].item_float_value = .5;// "Decay main ") ;  
            menu[4].item_float_value = 3;// "Fmod/Fmain ") ;
            menu[5].item_float_value = .25;// "FM depth max ") ;
            menu[6].item_float_value = .01;// "Attack FM ") ;
            menu[7].item_float_value = .1;// "Sustain FM ") ;
            menu[8].item_float_value = .4;// "Decay FM ") ;
            menu[9].item_float_value = 0;// "Lin=1/Quad DK ") ;
            menu[10].item_float_value = 1;//  "Run ") ;
        }


        PT_YIELD_usec(10000);
    }    

    PT_END(pt);
} // check buttons thread


static PT_THREAD(protothread_readmux(struct pt* pt))
{
    PT_BEGIN(pt);

    while (1) {

        // yield time 0.1 second
        PT_YIELD_usec(5000) ;
        // PT_YIELD_INTERVAL(100000);
        uint16_t result1;
        uint16_t result2;

        adc_select_input(2);
        result1 = adc_read();
        
        adc_select_input(1);
        result2 = adc_read();


        //COMMENT OUT TO TRY PRESSING THROUGH SERIAL INSTEAD
        voltage1 = result1 * conversion_factor;
        voltage2 = result2 * conversion_factor;
        

        static int key1; 
        key1 = mux_select + 12; 
        static int key2; 
        key2 = mux_select + 16 + 12;
        // printf("MUX 1, voltage: %f V, on key %d\n", voltage1, key1);
        // printf("MUX 2, voltage: %f V, on key %d\n", voltage2, key2);

        prev_pressed[key1] = pressed[key1];
        prev_pressed[key2] = pressed[key2];
        if (mux_select > NUM_PHYSICAL_KEYS){
            voltage1 = 3;
            voltage2 = 3;
        }

        // checking if key on mux 1 is pressed 
        if (voltage1 < VOLTAGE_CUTOFF) {
            pressed[key1] = true;
        }
        else {
            pressed[key1] = false;
        }
        if (pressed[key1] && !prev_pressed[key1]) {
            play_note[key1] = true;
            current_main_inc[key1] = main_inc[key1];
            current_mod_inc[key1] = mod_inc[key1];
            note_start[key1] = true;
            printf("Adding %d\n", key1);
            add_note(key1);
        }

        // checking if key on mux 1 is pressed 
        if (voltage2 < VOLTAGE_CUTOFF) {
            pressed[key2] = true;
        }
        else {
            pressed[key2] = false;
        }
        if (pressed[key2] && !prev_pressed[key2]) {
            play_note[key2] = true;
            current_main_inc[key2] = main_inc[key2];
            current_mod_inc[key2] = mod_inc[key2];
            note_start[key2] = true;
            printf("Adding %d\n", key2);
            add_note(key2);
        }

        //COMMENT OUT TO TRY PRESSING THROUGH SERIAL INSTEAD
        mux_select++;
        mux_select = mux_select >= NUM_PHYSICAL_KEYS ? 0 : mux_select;

        gpio_put(MUX_SEL_A, mux_select >> 0 & 0b1);
        gpio_put(MUX_SEL_B, mux_select >> 1 & 0b1);
        gpio_put(MUX_SEL_C, mux_select >> 2 & 0b1);
        gpio_put(MUX_SEL_D, mux_select >> 3 & 0b1);
        //
        // NEVER exit while

    } // blink thread
            // NEVER exit while

    PT_END(pt);
} // blink thread

// ==================================================
// === FM parameter setup core1
// ==================================================
// 
static PT_THREAD(protothread_FM(struct pt* pt))
{
    PT_BEGIN(pt);

    // convert alarm period in uSEc to rate
    Fs = 1.0 / ((float)alarm_period * 1e-6);
    //
    while (1) {

        // == Fout and Fmod are in Hz
        // == fm_depth is 0 to 10 or so
        // == times are in seconds
        // wait for the run command
        PT_YIELD_UNTIL(pt, menu[10].item_float_value == 1);

        // conversion to intrnal units
        // increment = Fout/Fs * 2^32

        Fmod = menu[4].item_float_value;

        float current_note;
        for (int i = 0; i < NUM_KEYS; i++) {
            current_note = notes[i];
            main_inc[i] = current_note * pow(2, 32) / Fs;
            mod_inc[i] = Fmod * current_note * pow(2, 32) / Fs;
        }
        
        // fm modulation strength
        max_mod_depth = float_to_fix(menu[5].item_float_value * 100000);

        // convert main input times to sample number
        attack_time = float_to_fix(menu[1].item_float_value * Fs);
        decay_time = float_to_fix(menu[3].item_float_value * Fs);
        sustain_time = float_to_fix(menu[2].item_float_value * Fs);
        // and now get increments
        attack_inc = div(max_amp, attack_time);
        // linear and parabolic fit
        decay_inc = div(max_amp, decay_time);
        recip_decay_time = div(onefix, decay_time);
        //quad_decay_inc = mul((decay_inc<<1), recip_decay_time);
        // this need to be floating point becuase of squareing the decay time
        //f_quad_decay_inc = (fix_to_float(decay_inc<<1)/fix_to_float(decay_time));

        // convert modulation input times to sample number
        mod_attack_time = float_to_fix(menu[6].item_float_value * Fs);
        mod_decay_time = float_to_fix(menu[8].item_float_value * Fs);
        mod_sustain_time = float_to_fix(menu[7].item_float_value * Fs);
        // and now get increments
        // precomputing increments means that only add/subtract is needed
        mod_attack_inc = div(max_mod_depth, mod_attack_time);
        mod_decay_inc = div(max_mod_depth, mod_decay_time);
        if (printParams) {
            printParams = false;
            printf("--------------------------------------------\n"
                "octave_num: %f\nFmod: %f\nattack_time: %f\ndecay_time: %f\n"
                "sustain_time: %f\nattack_inc: %f\ndecay_inc: %f\nmod_attack_time: %f\n"
                "mod_decay_time: %f\nmod_sustain_time: %f\nmod_depth: %f\n",
                (octave_num), (Fmod), menu[1].item_int_value, menu[3].item_int_value,
                menu[2].item_int_value, fix_to_float(attack_inc), fix_to_float(decay_inc), menu[6].item_int_value,
                menu[8].item_int_value, menu[7].item_int_value, fix_to_float(max_mod_depth));
        }
        // tell the synth ISR to go
        static int i, j;


      // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // timer thread

// ==================================================
// === ISR routine -- RUNNING on core 1
// ==================================================
// 
void compute_sample(void)
{
    // === 
    static int i;
    for (int j = 0; j < BUFFER_COUNT; j++) {
        // start a burst on new data
        i = buffer[j]; 
        //i = j;
        if (i != -1) {
        if (note_start[i]) {
            // reset the start flag
            note_start[i] = false;
            // init the amplitude
            current_amp[i] = attack_inc;
            current_mod_depth[i] = mod_attack_inc;
            // reset envelope time
            note_time[i] = 0;
            // phase lock the main frequency
            main_accum[i] = 0;
            add_delay[i] = 0;
        } // note start
        else if (pressed[i] && prev_pressed[i]) {
            add_delay[i] += onefix;
        }
        // play the burst as long as the amplitude is positive
        // as it decays linearly
        if (current_amp[i] > 0) {

            // update dds modulation freq
            mod_accum[i] += current_mod_inc[i];
            mod_wave[i] = sine_table[mod_accum[i] >> 24];
            // get the instataneous modulation amp 
            // update modulation amplitude envelope 
            // printf("mod attack time is %f\n", fix_to_float(mod_attack_time));
            if (note_time[i] < (mod_attack_time + mod_decay_time + mod_sustain_time + add_delay[i])) {
                current_mod_depth[i] = (note_time[i] <= mod_attack_time) ?
                    current_mod_depth[i] + mod_attack_inc :
                    (note_time[i] <= mod_attack_time + mod_sustain_time + add_delay[i]) ? current_mod_depth[i] :
                    current_mod_depth[i] - mod_decay_inc;
            }
            else {
                current_mod_depth[i] = 0;
            }

            // set dds main freq and FM modulate it
            main_accum[i] += current_main_inc[i] + (unsigned int)mul(mod_wave[i], current_mod_depth[i]);
            // update main waveform
            main_wave[i] = sine_table[main_accum[i] >> 24];

            // get the instataneous amp 
            // update amplitude envelope 
            // linear EXCEPT for optional parabolic decay
            if (note_time[i] < (attack_time + decay_time + sustain_time + add_delay[i])) {
                if (note_time[i] <= attack_time) current_amp[i] += attack_inc;
                else if (note_time[i] > attack_time + sustain_time + add_delay[i]) {
                    if (linear_dk == 1) { current_amp[i] -= decay_inc; }
                    else {
                        current_amp[i] = current_amp[i] - (decay_inc << 1) +
                            div(mul((decay_inc << 1), (note_time[i] - attack_time - sustain_time - add_delay[i])), decay_time);
                    }
                }
            }
            else {
                current_amp[i] = 0;
            }
            // amplitide modulate and shift to the correct range for PWM
            main_wave[i] = mul(main_wave[i], current_amp[i]);

            // move time ahead
            note_time[i] += onefix;
        }
        }

    }

    fix sum_waves = int_to_fix(0);

    for (int i = 0; i < BUFFER_COUNT; i++) {
        int j = buffer[i];
        if (j != -1) {
            sum_waves += main_wave[j];
        }
    }

    fix final_wave = div(sum_waves, int_to_fix(BUFFER_COUNT));
    //printf("final wave = %d\n", fix_to_int(final_wave));

    DAC_data = (DAC_config_chan_A | ((fix_to_int(final_wave) + 2048) & 0xfff));

    // Write data to DAC
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

} // end ISR call

void add_note(int note) {
    // either the note is already present in buffer, or it is not present 

    // look for index if it is present. 
    int index = -1;
    for (int i = 0; i < BUFFER_COUNT; i++) {
        if (buffer[i] == note) {
            index = i;
        }
    }
    // if not found, remove the first element of buffer and then add new note to 
    // end. 
    int next;
    if (index == -1) {
        
        for (int j = 0; j < BUFFER_COUNT-1; j++) {
            next = buffer[j+1]; 
            buffer[j] = next;
        }
        // note is now at end of buffer, most recently played. 
        buffer[BUFFER_COUNT - 1] = note;
    }
    // if found, shift notes so that the note is now at end of buffer 
    // do not need to remove any elements of buffer since there 
    // is already space allocated for note. 
    else {
        for (int k = index; k < BUFFER_COUNT-1; k++) {
            next = buffer[k+1]; 
            buffer[k] = next; 
        }
        buffer[BUFFER_COUNT - 1] = note;
    }
}

// debugging buffer logic
void print_notes(void) {
    printf("buffer :");
    for (int i = 0; i < BUFFER_COUNT; i++) {
        printf("%d, ", buffer[i]);
    }
    printf("\n");
}

// User input thread. 
static PT_THREAD(protothread_serial(struct pt* pt))
{
    PT_BEGIN(pt);
    static char classifier;
    static int test_in;
    static float float_in;
    printParams = false;
    static int key = 0;
    

    while (1) {
        classifier = 0;
        test_in = 0;
        float_in = 0.0;
        static char user_input_string[40];

        sprintf(pt_serial_out_buffer, ": ");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number

        // sscanf(pt_serial_in_buffer, "%d %f", &test_in, &float_in);
        if ((test_in > 8 || test_in < 1)) {
            sscanf(pt_serial_in_buffer, "%s %f", &user_input_string, &float_in);

            printParams = true;
            if (!strcmp(user_input_string, "mainmod")) {
                change_value_serial(4, float_in);
            }
            else if (!strcmp(user_input_string, "mainatk")) {
                change_value_serial(1, float_in);
            }
            else if (!strcmp(user_input_string, "modatk")) {
                change_value_serial(6, float_in);
            }
            else if (!strcmp(user_input_string, "mainsus")) {
                change_value_serial(2, float_in);
            }
            else if (!strcmp(user_input_string, "modsus")) {
                change_value_serial(7, float_in);
            }
            else if (!strcmp(user_input_string, "maindk")) {
                change_value_serial(3, float_in);
            }
            else if (!strcmp(user_input_string, "moddk")) {
                change_value_serial(8, float_in);
            }
            else if (!strcmp(user_input_string, "octave")) {
                change_value_serial(0, float_in);
            }
            else if (!strcmp(user_input_string, "moddepth")) {
                change_value_serial(5, float_in);
            }
            else if (!strcmp(user_input_string, "scale")) {
                int tn;
                for (tn = 0; tn < NUM_KEYS; tn++) {
                    sprintf(pt_serial_out_buffer, "playing note %d", tn);
                    serial_write;
                    current_main_inc[tn] = main_inc[tn];
                    current_mod_inc[tn] = mod_inc[tn];
                    note_start[tn] = true;
                    PT_YIELD_usec(1000000);

                    // PT_YIELD_UNTIL(pt, current_amp[i-1]<onefix);
                    // PT_YIELD_usec(10000);
                    // play_note[i] = false; 
                }
            }


            else {
                sprintf(pt_serial_out_buffer,
                    "Command '%s' could not be recognized\n\r>>>",
                    user_input_string);
                serial_write;

            }
        }
        else {
            mux_select = test_in - 1;
            sprintf(pt_serial_out_buffer,
                "Setting key '%d' to %f volts\n\r>>>",
                test_in, float_in);
            serial_write;
            voltage1 = float_in;
        }

    }
    PT_END(pt);
}

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main() {

    // fire off interrupt
    alarm_in_us(alarm_period);

    //  === add threads  ====================
    // for core 1
    pt_add_thread(protothread_FM);
    //pt_add_thread(protothread_fft);
    
    //
    // === initalize the scheduler ==========
    pt_schedule_start;
    // NEVER exits
    // ======================================
}



// ========================================
// === core 0 main
// ========================================
int main() {
    // set the clock
    //set_sys_clock_khz(250000, true); // 171us

    for (int i = 0; i < NUM_KEYS; i++) {
        current_amp[i] = float_to_fix(2000.0);
    }

    for (int i = 0; i < NUM_KEYS; i++) {
        note_start[i] = true;
        play_note[i] = false; // no keys pressed initially
    }

    for (int i = 0; i < NUM_SONGS; i++) {
        play_song[i] = false; // no songs to be played initially
    }

    for (int i = 0; i < BUFFER_COUNT; i++) {
        buffer[i] = -1; // no keys pressed initially
    }

    for (int i = 0; i < NUM_KEYS; i++) {
        notes[i] = 440.0 * pow(2, (base_note+i-69.0)/12.0);
    }


    // dds table 10 bit values
    int i = 0;
    while (i < 256) {
        // sine table is in naural +1/-1 range
        sine_table[i] = float_to_fix(sin(2 * 3.1416 * i / 256));
        i++;
    }

    // start the serial i/o
    // stdio_init_all();
    // announce the threader version on system reset
    printf("\n\rProtothreads RP2040 v1.11 two-core\n\r");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    // connected to spi DAC
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    gpio_init(MUX_SEL_A);
    gpio_init(MUX_SEL_B);
    gpio_init(MUX_SEL_C);
    gpio_init(MUX_SEL_D);

    gpio_set_dir(MUX_SEL_A, GPIO_OUT);
    gpio_set_dir(MUX_SEL_B, GPIO_OUT);
    gpio_set_dir(MUX_SEL_C, GPIO_OUT);
    gpio_set_dir(MUX_SEL_D, GPIO_OUT);

    gpio_put(MUX_SEL_A, 0);
    gpio_put(MUX_SEL_B, 0);
    gpio_put(MUX_SEL_C, 0);
    gpio_put(MUX_SEL_D, 0);

    for(int i = 0; i < NUM_SONGS; i++) {
        gpio_init(song_buttons[i]);
        gpio_set_dir(song_buttons[i], GPIO_IN);
        gpio_pull_up(song_buttons[i]);
    }

    for(int i = 0; i < NUM_INSTRUMENTS; i++) {
        gpio_init(instrument_buttons[i]);
        gpio_set_dir(instrument_buttons[i], GPIO_IN);
        gpio_pull_up(instrument_buttons[i]);
    }
    
    //
    sprintf(menu[0].item_name, "Octave # ");
    sprintf(menu[1].item_name, "Attack main ");
    sprintf(menu[2].item_name, "Sustain main ");
    sprintf(menu[3].item_name, "Decay main ");
    sprintf(menu[4].item_name, "Fmod/Fmain ");
    sprintf(menu[5].item_name, "FM depth max ");
    sprintf(menu[6].item_name, "Attack FM ");
    sprintf(menu[7].item_name, "Sustain FM ");
    sprintf(menu[8].item_name, "Decay FM ");
    sprintf(menu[9].item_name, "Lin=1/Quad DK ");
    sprintf(menu[10].item_name, "Run ");
    //
    menu[0].item_data_type = 0;// int
    menu[1].item_data_type = 1;// float
    menu[2].item_data_type = 1;// float
    menu[3].item_data_type = 1;// float
    menu[4].item_data_type = 1;// float
    menu[5].item_data_type = 1;// float
    menu[6].item_data_type = 1;// float
    menu[7].item_data_type = 1;// float
    menu[8].item_data_type = 1;// float
    menu[9].item_data_type = 0;// int
    menu[10].item_data_type = 0;// int
    //
    menu[0].item_inc_type = 0;// lin update
    menu[1].item_inc_type = 1;// log update
    menu[2].item_inc_type = 1;// log
    menu[3].item_inc_type = 1;// log
    menu[4].item_inc_type = 0;// lin update
    menu[5].item_inc_type = 1;// log update
    menu[6].item_inc_type = 1;// log
    menu[7].item_inc_type = 1;// log
    menu[8].item_inc_type = 1;// log
    menu[9].item_inc_type = 0;// 
    menu[10].item_inc_type = 0;// 
    //
    menu[0].item_increment = 1;   //"Octave # ") ;
    menu[1].item_increment = 0.99;  //"Attack main ") ;
    menu[2].item_increment = 1.1;  //"Sustain main ") ;
    menu[3].item_increment = 0.98;   //"Decay main ") ;  
    menu[4].item_increment = 0.01;   //"Fmod/Fmain ") ;
    menu[5].item_increment = 3.0;  //"FM depth max ") ;
    menu[6].item_increment = 0.95;  //"Attack FM ") ;
    menu[7].item_increment = 1.1;   //"Sustain FM ") ;
    menu[8].item_increment = 0.97;   //"Decay FM ") ;
    menu[9].item_increment = 1;  //"Lin=1/Quad DK ") ;
    menu[10].item_increment = 1;  // "Run ") ;

    // piano configuration
    menu[0].item_float_value = 3;// "Octave # ") ;
    menu[1].item_float_value = .01;// "Attack main ") ;
    menu[2].item_float_value = .3;// "Sustain main ") ;
    menu[3].item_float_value = .5;// "Decay main ") ;  
    menu[4].item_float_value = 3;// "Fmod/Fmain ") ;
    menu[5].item_float_value = .25;// "FM depth max ") ;
    menu[6].item_float_value = .01;// "Attack FM ") ;
    menu[7].item_float_value = .1;// "Sustain FM ") ;
    menu[8].item_float_value = .4;// "Decay FM ") ;
    menu[9].item_float_value = 0;// "Lin=1/Quad DK ") ;
    menu[10].item_float_value = 1;//  "Run ") ;

    menu[0].item_float_min = 1;
    menu[0].item_float_max = 6;
    menu[1].item_float_min = .001;
    menu[1].item_float_max = 5;
    menu[2].item_float_min = .001;
    menu[2].item_float_max = 5;
    menu[3].item_float_min = .001;
    menu[3].item_float_max = 5;
    menu[4].item_float_min = .001;
    menu[4].item_float_max = 100;
    menu[5].item_float_min = .001;
    menu[5].item_float_max = 100;
    menu[6].item_float_min = .001;
    menu[6].item_float_max = 5;
    menu[7].item_float_min = .001;
    menu[7].item_float_max = 5;
    menu[8].item_float_min = .001;
    menu[8].item_float_max = 5;
    menu[9].item_float_min = 0;
    menu[9].item_float_max = 1;
    menu[10].item_float_min = 0;
    menu[10].item_float_max = 1;

    // init the ADC
    ADC_setup();

    // start core 1 threads
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // === config threads ========================
    // for core 0

    pt_add_thread(protothread_readmux);
    pt_add_thread(protothread_buttonpress);
    // pt_add_thread(protothread_serial);
    pt_add_thread(protothread_playsong);
    //
    // === initalize the scheduler ===============
    pt_schedule_start;
    // NEVER exits
    // ===========================================
} // end main