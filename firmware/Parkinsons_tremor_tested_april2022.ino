/* Karthik Madhav Jain
 * The Pennsyvania State University, University Park
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <P.D._Tremor_Suppression_inferencing.h> //Inferencing header file
#include <Arduino_LSM9DS1.h> //IMU header file
#include <Wire.h>
#include "Adafruit_DRV2605.h" //Haptic controller header file

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal //Edit: Try setting to true (in old version) and seeing what features are generated
static uint32_t run_inference_every_ms = 200;
static rtos::Thread inference_thread(osPriorityLow);
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

char last_prediction[20];
char current_prediction[20];

Adafruit_DRV2605 drv;

float instantaneous_magnitude_of_acceleration = 0;

/* Forward declaration */
void run_inference_background();

/**
* @brief      Arduino setup function
*/


void setup()
{
    // put your setup code here, to run once:
    
    Serial.begin(115200);

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 6) { //Edit: Might need to change it to 3 as we may not be using gyroscope measurements anymore
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 6 (the 6 sensor axes)\n");
        return;
    }
    
    drv.begin();
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_INTTRIG);

    inference_thread.start(mbed::callback(&run_inference_background));
}

uint8_t effect1 = 1; //Haptic effect for resting tremor //Edit: These effects are just the preliminary ones, further analysis of all available effects will help select the perfect ones for each case
uint8_t effect2 = 123; //Haptic effect for action tremor

/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...) {
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) {
       Serial.write(print_buf);
   }
}

/**
 * @brief      Run inferencing in the background.
 */
void run_inference_background()
{
    
    // wait until we have a full buffer
    delay((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT) + 100);
    
    // This is a structure that smoothens the output result
    // With the default settings 70% of readings should be the same before classifying. //Edit: Default settings may need to be tweaked to get more accurate results
    ei_classifier_smooth_t smooth;
    ei_classifier_smooth_init(&smooth, 10 /* no. of readings */, 7 /* min. readings the same */, 0.8 /* min. confidence */, 0.3 /* max anomaly */);
    while (1) {        
        //drv.setWaveform(0, effect1); 
        //drv.setWaveform(1, 0);

        // copy the buffer
        memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));
        
        // Turn the raw buffer in a signal which we can the classify
        signal_t signal;
        int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }

        
        // Run the classifier
        ei_impulse_result_t result = { 0 };

        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }

        /*float max_peak_height = 0;
        float min_peak_height = 0;
          //float magnitude_of_acceleration = 0;
          //float peak_difference = 0;
        for(int i=0; i<((sizeof(inference_buffer))/sizeof(inference_buffer[0]));i=i+6){

            //For the first 3 of every 6 values of the inference buffer (accelerometer values), calculate the magnitude of acceleration
          float magnitude_of_acceleration = sqrt(sq(inference_buffer[i])+sq(inference_buffer[i+1])+sq(inference_buffer[i+2])); //Magnitude of acceleration [0,1]
          ei_printf("\nThe mag_acc is: %s", magnitude_of_acceleration);
            //Find maximum peak height, minimum peak height and range of values (peak difference)
          if(max_peak_height<= magnitude_of_acceleration){
            max_peak_height = magnitude_of_acceleration;
          }
          if(min_peak_height>= magnitude_of_acceleration){
            min_peak_height = magnitude_of_acceleration;
          }
        }
        float peak_difference = max_peak_height-min_peak_height;
        //ei_printf("\nThe peak difference is: %s", peak_difference);
        //ei_printf("\nInstantaneous Magnitude of Acceleration: %s", instantaneous_magnitude_of_acceleration);*/
        
        // ei_classifier_smooth_update yields the predicted label
        const char *prediction = ei_classifier_smooth_update(&smooth, &result);
        ei_printf("\nThe current prediction is: %s ", prediction);
        ei_printf("   and the last prediction was: %s", last_prediction);
        strcpy(current_prediction, prediction);
      
        //Condition for sending Haptic 'Alert" Signal (resting tremors)
        /*if(((strcmp(last_prediction, "idle")==0) || (strcmp(last_prediction, "pill_rolling_tremor")==0)) && (strcmp(current_prediction, "pill_rolling_tremor")==0)){ //Edit: Check to see if this condition satisfies all requirements or if a confirmation of return to idle/voluntary motion state is required  
          ei_printf("\nPill Rolling Tremor Haptic Feedback GO");
          drv.setWaveform(0, effect1); 
          drv.setWaveform(1, 0);
          drv.go();
          delay(100);
        }*/

        //Condition for sending Haptic 'Noise' Signal (action tremors)
        /*else*/ //if(((strcmp(last_prediction, "pill_rolling_tremor")==0) || (strcmp(last_prediction, "pill_rolling_tremor")==0)) && (strcmp(current_prediction, "pill_rolling_tremor")==0)){ 
        /*else*/ if(((strcmp(last_prediction, "voluntary_motion")==0) || (strcmp(last_prediction, "voluntary_motion")==0)) && (strcmp(current_prediction, "voluntary_motion")==0)){ 

          ei_printf("\nTremor Motion Haptic Feedback GO");
          drv.setWaveform(0, random(1,123)); 
          drv.setWaveform(1, 0);
          drv.go();
          delay(200);
          /*float max_peak_height = 0;
          float min_peak_height = 0;
          //float magnitude_of_acceleration = 0;
          //float peak_difference = 0;
          for(int i=0; i<((sizeof(inference_buffer))/sizeof(inference_buffer[0]));i=i+6){

            //For the first 3 of every 6 values of the inference buffer (accelerometer values), calculate the magnitude of acceleration
            float magnitude_of_acceleration = sqrt(sq(inference_buffer[i])+sq(inference_buffer[i+1])+sq(inference_buffer[i+2])); //Magnitude of acceleration [0,1]
            ei_printf("\nThe mag_acc is: %s", magnitude_of_acceleration);
            //Find maximum peak height, minimum peak height and range of values (peak difference)
            if(max_peak_height<= magnitude_of_acceleration){
              max_peak_height = magnitude_of_acceleration;
            }
            if(min_peak_height>= magnitude_of_acceleration){
              min_peak_height = magnitude_of_acceleration;
            }
          }
          float peak_difference = max_peak_height-min_peak_height;
          ei_printf("\nThe peak difference is: %s", peak_difference);
          ei_printf("\nInstantaneous Magnitude of Acceleration: %s", instantaneous_magnitude_of_acceleration);
           
          
          ei_printf("\nThe peak difference is: %s", peak_difference);
          ei_printf("\nInstantaneous Magnitude of Acceleration: %s", instantaneous_magnitude_of_acceleration);
          if((instantaneous_magnitude_of_acceleration <= 0.8*peak_difference) && (instantaneous_magnitude_of_acceleration >= 0.2*peak_difference)){ //Edit: Arbitrary constraints of 80% and 20% used. Further analysis required for best constraint amount
            Serial.print("\nWrist flexion Tremor Haptic Feedback GO");
            drv.go(); //Edit: Currently, this effect will only trigger when the magnitude of acceleration is not at the peak values. (Might need to change to simply not maximum value). Amount of time the signal is passed could also be dependent on the value of mag_acc=> PWM input for haptic controller will need to be figured out. OR simply the effect could be changed up based on the frequency of the signal, and where in the signal we are
            delay(100);
          }*/
           
        }
        
        strcpy(last_prediction, prediction); //copy prediction value to global var
        delay(run_inference_every_ms);
    }
    ei_classifier_smooth_free(&smooth);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/

void loop()
{
    while (1) {

        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
       
        // roll the buffer -6 points so we can overwrite the last one
        numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -6); //Edit: buffer roll, buffer write operations may need to be changed if gyroscope is no longer used
        
        // read to the end of the buffer
        IMU.readAcceleration(
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 5],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 6],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 4]
        );
        IMU.readGyroscope(
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2],
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1]
        );

        instantaneous_magnitude_of_acceleration = sqrt(sq(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 6])+sq(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 5])+sq(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 4])); //Edit: Magnitude of acceleration may not be the best metric for processing peaks and other features in the signal
        
        // and wait for next tick
        uint64_t time_to_wait = next_tick - micros();
        delay((int)floor((float)time_to_wait / 1000.0f));        
        delayMicroseconds(time_to_wait % 1000);
    }
}
