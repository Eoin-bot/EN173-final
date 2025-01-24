#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <nrfx_saadc.h>

LOG_MODULE_REGISTER(final, LOG_LEVEL_DBG);
#define LED0_NI DT_ALIAS(led0)
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NI, gpios);
#define LED1_NI DT_ALIAS(led1)
const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NI, gpios);
#define BUTTON1_NI DT_ALIAS(sw0)
const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BUTTON1_NI, gpios);
//#define TMP36 DT_NODELABEL(temp0)
//const struct adc_dt_spec tmp36 = ADC_DT_SPEC_GET(TMP36);
#define NUM_ADC_READINGS    10
 


#define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN0
static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);//the temperature input


#define SAMPLE_INTERVAL_MS 5000
static void temp_counter_handler(struct k_timer * timer);
K_TIMER_DEFINE(temp_timer, temp_counter_handler, NULL);

volatile int tempCount;
#define STORE_SIZE 12
float storedTemperatures[STORE_SIZE]; 

static uint16_t buf;

void temp_counter_handler(struct k_timer *timer) {
    //uint16_t buf; 
    //int val_mV;
    //float avg_mV; 
    float T_in_C;


        nrfx_err_t err = nrfx_saadc_mode_trigger();
        if (err != NRFX_SUCCESS) {
	    printk("nrfx_saadc_mode_trigger error: %08x", err);
	    return;
    
    printk("TIMER WENT OFF");
        }
        int avg_mV = ((600*2)*buf) / ((1<<12)); 
                
        T_in_C = 25.0 + (avg_mV - 750) / 10;
        storedTemperatures[tempCount % STORE_SIZE] = T_in_C;
        tempCount++;
          
    
}

static void configure_saadc(void) {


/* STEP 5.1 - Connect ADC interrupt to nrfx interrupt handler */
        IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)), DT_IRQ(DT_NODELABEL(adc), priority), nrfx_isr, nrfx_saadc_irq_handler, 0);
        
        /* STEP 5.2 - Initialize the nrfx_SAADC driver */
        nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
        if (err != NRFX_SUCCESS) 
        {
                printk("nrfx_saadc_mode_trigger error: %08x", err);
                return;
        }

        /* STEP 5.3 - Configure the SAADC channel */
        channel.channel_config.gain = NRF_SAADC_GAIN1_2;
        err = nrfx_saadc_channels_config(&channel, 1);
        if (err != NRFX_SUCCESS) 
        {
		printk("nrfx_saadc_channels_config error: %08x", err);
	        return;
	}

        /* STEP 5.4 - Configure nrfx_SAADC driver in simple and blocking mode */
        err = nrfx_saadc_simple_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
        if (err != NRFX_SUCCESS) {
                printk("nrfx_saadc_simple_mode_set error: %08x", err);
                return;
        }
        
        /* STEP 5.5 - Set buffer where sample will be stored */
        err = nrfx_saadc_buffer_set(&buf, 1);
        if (err != NRFX_SUCCESS) {
                printk("nrfx_saadc_buffer_set error: %08x", err);
                return;
        }
        k_timer_start(&temp_timer, K_MSEC(SAMPLE_INTERVAL_MS), K_MSEC(SAMPLE_INTERVAL_MS));
}

int main(void)
{
    tempCount = 0;
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&btn, GPIO_INPUT);
    configure_saadc();
    int prev_btn, curr_btn;
    prev_btn = gpio_pin_get_dt(&btn);

    while (true) {
        float sum = 0;
        float avg = 0;
        curr_btn = gpio_pin_get_dt(&btn);
        if ((curr_btn) && (curr_btn != prev_btn)) {
            LOG_DBG("Button pressed");
            int num_to_avg;
            if (tempCount <= STORE_SIZE) {
                num_to_avg = tempCount;
            } else {
                num_to_avg = 12;
            }
            if (tempCount != 0) {
                sum = 0;
                for(int i = 0; i < num_to_avg; i++) {
                    sum = sum + storedTemperatures[i];
                    LOG_INF("Temperaure measured = %.1f deg C", storedTemperatures[i]);
                }
                avg = sum / num_to_avg;
                LOG_INF(" Avg Temperature = %.01f deg C", avg);
            }
            tempCount = 0;
        }
        if (tempCount > STORE_SIZE) {
            gpio_pin_set_dt(&led1, 1);
        } else {
            gpio_pin_set_dt(&led1, 0);
        }
        k_msleep(100);
        prev_btn = curr_btn; 
    }
        //return 0;
}