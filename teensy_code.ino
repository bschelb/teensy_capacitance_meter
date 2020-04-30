#include "Filter.h"
#include <math.h>
#define ToNano Serial1
#define ToNano2 Serial2

// 20 is the weight (20 => 20%)
ExponentialFilter<double> ADCFilter(20,0);

#define TOUCH_PIN (0)
#define CAL_TOUCH_PIN1 (15)
#define CAL_TOUCH_PIN2 (17)
#define CAL_TOUCH_PIN3 (18)
#define led_pin (13)
double m,b_quad,b_linreg,a,c,quad_accuracy,linreg_accuracy,capacitance;
float old_capacitance,delta;
int sent_capacitance,sent_delta;


void setup()
{ 
    // **ALL CAPACITORS MUST BE CONNECTED TO GROUND ON TEENSY**
    pinMode(TOUCH_PIN, INPUT); // Actual capacitor under test to be measured
    pinMode(CAL_TOUCH_PIN1, INPUT); // 202 pF known calibrating capcitance
    pinMode(CAL_TOUCH_PIN2, INPUT); // 404 pF known calibrating capacitance
    pinMode(CAL_TOUCH_PIN3, INPUT); // 0 pF known calibrating capacitance
    
//    ARM_DEMCR |= ARM_DEMCR_TRCENA; // This line and the next line allow the counting of clock cycles in the CPU
//    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; // This can be coupled with a Teensy library function to calculate nano-seconds

    double avg_touch_pin1; // Next 14 lines average out the readings for calibration
    double avg_touch_pin2;
    double avg_touch_pin3;
    for(int i=150; i>0; i--)
       {   
         avg_touch_pin1+=touchRead(CAL_TOUCH_PIN1);
         avg_touch_pin2+=touchRead(CAL_TOUCH_PIN2);
         avg_touch_pin3+=touchRead(CAL_TOUCH_PIN3);
         delay(30);
       }
    avg_touch_pin1 = avg_touch_pin1 / 150;
    avg_touch_pin2 = avg_touch_pin2 / 150;
    avg_touch_pin3 = avg_touch_pin3 / 150;

    c = avg_touch_pin3; // Auto-calibration based on quadratic equation this line + next 4 lines
    double sys1 = avg_touch_pin1 - c;
    double sys2 = avg_touch_pin2 - c;
    a = (((sys1 * -2) / sys2) / 81608);
    b_quad = ((sys1 - (a * 40804)) / 202);
    quad_accuracy = abs(((-b_quad + sqrt((pow(b_quad, 2))-(4 * (a * (c - avg_touch_pin3))))) / (2 * a)) + 
    (((-b_quad + sqrt((pow(b_quad, 2))-(4 * (a * (c - avg_touch_pin1))))) / (2 * a)) - 202) + 
    (((-b_quad + sqrt((pow(b_quad, 2))-(4 * (a * (c - avg_touch_pin2))))) / (2 * a)) - 404));

    double top_m_sum_xy = (((0 * avg_touch_pin3) + // Calibration using linear regression next 4 lines
    (202 * avg_touch_pin1) + (404 * avg_touch_pin2)) * 3);
    double top_m_sum_y = (avg_touch_pin2 + avg_touch_pin1 + avg_touch_pin3);
    m = ((top_m_sum_xy - (606 * top_m_sum_y)) / 244824);
    b_linreg = ((top_m_sum_y - (m * 606)) / 3);
    linreg_accuracy = abs(((avg_touch_pin3 - b_linreg) / m) +
    (((avg_touch_pin1 - b_linreg) / m) - 202) +
    (((avg_touch_pin2 - b_linreg) / m) - 404));
    
    Serial.begin(9600); // Begins normal Serial output
    ToNano.setTX(1); // Sets the first transmit pin to use to transmit to the Nano 33 IoT
    ToNano.begin(9600); // Sets the first serial output to the Nano 33 IoT
    ToNano2.setTX(10); // Sets the second transmit pin to use to transmit to the Nano 33 IoT
    ToNano2.begin(9600); // Sets the second serial output to the Nano 33 IoT
}

void loop()
{
     ADCFilter.Filter(touchRead(TOUCH_PIN));

     if (quad_accuracy <= linreg_accuracy)
     {
        capacitance = ((-b_quad + sqrt((pow(b_quad, 2))-(4 * (a * (c - ADCFilter.Current()))))) / (2 * a));
     }
     else
     {
        capacitance = ((ADCFilter.Current() - b_linreg) / m); // Linear regression output
     }

     if (capacitance < 0.2)
     {
        capacitance = 0;
     }
     if (old_capacitance)
     {
        delta = - (old_capacitance - capacitance);
     }
     
     old_capacitance = capacitance;
     
     Serial.print("Raw reading: ");
     Serial.println(ADCFilter.Current());
     Serial.print(F("Capacitance Value = "));
     Serial.print(capacitance, 3);
     Serial.println(F(" pF "));
     Serial.print(F("Change in last known capacitance: "));
     Serial.println(delta, 3);
     
     sent_capacitance = capacitance;
     sent_delta = delta;
     ToNano.write(sent_capacitance);
     ToNano2.write(sent_delta);
     while (millis() % 500 != 0);
}
