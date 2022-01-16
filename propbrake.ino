/*

Propshaft brake controller
Copyright: Frans Veldman, 2015

Functionality:
Lock and Release switch: This is typically implemented as a momentary toggle switch with three positions: Lock, neutral, Release.
These switches have three actions:
Short press >25ms, <250ms.  In enhanced indication mode, a short press lights up the indication light but does not run the brake motor. In other modes it is equal to a middle press.
Normal press, shorter than the time to lock or release the brake. This activates the brake motor and runs it until either the current limit is reached or the maximum run time.
Prolonged press.  A prolonged press is ignored, except when SETUP_MODE or OVERRIDE_MODE is selected. In these cases the brake motor is activated as long as the switch is pressed.

Indication LED
The indication light has two modes.
A simple mode, where the light off indicates that the brake is free, and the light on indicates that the brake is locked.
An enhanced mode, where the light is normally off but shows the status when the toggle switch is briefly toggled in either direction: 
    A short flash indicates that the brake is free, a prolonged flash indicates that the brake is locked.
The purpose of the enhanced mode is to avoid keeping the indication light on all the time during prolonged sailing trips.
The mode is choosen by the INDICATION_MODE input: the default is normal mode, by connecting this input to ground enhanced mode is selected.
When the brake motor is activated in either direction, the LED flashes rapidly.

OVERRIDE_MODE
By default, the controller will only run the brake motor to the configured limits. Once the controller "thinks" it has set the brake either free or locked, prolonged or repeatedly pressing on the associated button will have no effect.
Also, when the controller detects that the engine is running, pressing the brake button will not activate the brake. This to avoid accidential engagement of the brake.
In override mode, the controller will work normally, but if the lock or release button is still pressed when the controller wants to stop the brake motor it will continue to run the brake motor. It will also run the brake motor again 
even if the brake has already been set in the choosen direction. 
Note that this will likely break the automatic setting of the brake: If the brake has moved for prolonged time in the release direction, then on the next automatic lock attempt the maximum duration will expire before the brake pads touch the disc.
Of course since you are in override mode anyway you can just hold the button until the brake engages.
Also, in this mode, it is possible to burn or lock up the brake motor, making subsequent release impossible with potentially dangerous results. 
Nevertheless, the brake can still be operated automatically as long as the buttons are not being pushed beyond the programmed time and current limits and not when the brake has already been set in the desired direction.

SETUP_MODE
Setup_mode is engaged by holding down the release button during power on.
In setup mode the controller will run the brake motor as long as the free or lock button is pressed and records the maximum current when the brake is being locked and the run time when the brake is freed. 
These values will be used during normal operation.

Technical:
The variable "run_duration" sets the time the motor should be run when the brake is released. Normally the brake will not hit its physical boundary during this time so it will not be limited by the maximum current setting.
When setting the brake, the motor is run a bit longer than run_duration, or until the current limit is reached. A slightly longer time is needed to lock the brake because the brake motor will labour more.
Normally the current limit will be reached when locking the brake before the duration is expired. This means that the brake is self adjusting.
Note that we measure current by measuring the voltage drop over a resistor in the positive power supply. Problem here is that any changes in power supply voltage will also change the voltage
measured over the resistor. So just before running the brake motor, we sample the voltage and use it as a reference. We may assume that in the short time the brake motor runs
the system voltage won't dramatically change.

The brake state is recorded in the EEPROM. This EEPROM has a life expectancy of 100.000 erase cycles. This should be sufficient for quite a while.
If the brake is operated on average 10 times per day, it will last for more than 30 years.
If the brake state is recorded wrong in case of a power failure between setting the brake and recording the new state, the following will happen:
If the brake state is falsely recorded as released, pressing the release button will not have effect (unless override mode is selected).
Attempts to lock the brake again will trigger the current limit almost immediately and the recorded state will now be recorded as locked. The brake can now be released again normally.
If the brake state is falsely recorded as locked, pressing the lock button will not have effect (unless override mode is selected).
Pressing the release button will run the brake motor and the brake pads will move further away from the disc.
On the next lock attempt(s) the brake will not run long enough to press the brake pads against the disc.
Since the brake motor will run longer in the lock direction than in the release direction (at least when the current limit doesn't kick in,
and it doesn't because the brake pads will not create resistance), the brake will move to its standard position again after a few lock-release cycles.
*/

#include <avr/eeprom.h>
// #include "LowPower.h"               // Uncomment this statement if you want to use the LowPower library. This reduces the power consumption from 16mA to 3mA. You have to install the LowPower library first!

// Settings: (All time values are in milliseconds)
#define DEBOUNCE            15         // Time to wait for the release and lock button to settle, to avoid rapid switching.
#define MAX_DURATION        4000       // Maximum configurable brake motor run duration.
#define MIN_DURATION        250        // Minimum configurable brake motor run duration.
#define DEFAULT_DURATION    1500       // Default brake motor run duration.
#define EXTRA_LOCK_DURATION 500        // Extra time when locking the brake. This is necessary because setting the brake takes longer, and to facilitate automatic brake adjustment.
#define TRANSITIONDELAY     10         // Delay between reversing the polarities of the brake motor. Too fast may cause a momentary short cut and an amp spike.
#define CURRENTDELAY        200        // Delay before paying attention to the motor current. This to ignore the start up surge of the brake motor.
#define SWITCH_SHOW         350        // Keypress less than this amount of milliseconds will show status rather than invoking a change of brake position.
#define LED_LONG            5000       // milliseconds to show status when brake is locked in enhanced indicator mode
#define LED_SHORT           300        // milliseconds to show status when brake is released in enhanced indicator mode
#define LED_FLASH           50         // milliseconds between changes of LED state while the brake motor is running
#define LED_SETUP           150        // milliseconds between changes of LED state while entering setup mode

// Digital Outputs:
#define LED_HI              13         // This output is HIGH when the indication light should be lit. This is the reverse of the Indication_LO output.
#define LED_LO              12         // This output is LOW when the indication light should be lit. This is the reverse of the Indication_HI output.
#define BRAKE_RUN           8          // This output is HIGH when the brake motor needs to run in either direction.
#define BRAKE_LOCK          7          // This output is HIGH when the brake motor needs to be activated in the lock direction.
#define BRAKE_RELEASE       9          // This output is HIGH when the brake motor needs to be activated in the release direction.

// Digital Inputs:
#define SW_LOCK             2          // This input is pulled to LOW when the brake needs to be locked. Typically, the lock switch is the upper half of a toggle switch with a neutral central position.
#define SW_RELEASE          3          // This input is pulled to LOW when the brake needs to be released. Typically, the release switch is the lower half of a toggle switch with a neutral central position.
#define ENGINE              4          // This input is pulled to HIGH when the engine power is switched on. Connect to ground if you don't want to use automatic brake release.
#define INDICATION_MODE     5          // This input is pulled to LOW to select enhanced indication mode.
#define OVERRIDE_MODE       6          // This input is pulled to LOW to select override mode.

// Analog Inputs:
#define CURRENT             1          // This input senses the brake motor current. The motor current is used to detect when the brake is locking up or hitting a physical boundary.

// Constants
#define LOCK                1
#define RELEASE             2


struct settings_t {
  byte state;                          // Recorded brake state.
  unsigned int run_duration;           // Maximum brake motor run duration.
  unsigned int max_current;            // Maximum brake motor current.
} settings;


byte switchstate=0;
byte brakemotorrunning=0;
boolean setup_mode=false;
boolean engine_released=false;
unsigned long debounce=0;
unsigned long switchchanged=0;
unsigned long led_timer=0;
unsigned long motorstop=0;
unsigned long motorstarted=0;


void setup() {
    // Initialize digital inputs
    pinMode(SW_LOCK,INPUT_PULLUP);
    pinMode(SW_RELEASE,INPUT_PULLUP);
    pinMode(INDICATION_MODE,INPUT_PULLUP);
    pinMode(OVERRIDE_MODE,INPUT_PULLUP);
    pinMode(ENGINE,INPUT);
    
    // Initialize digital outputs    
    pinMode(LED_HI,OUTPUT);
    pinMode(LED_LO,OUTPUT);
    pinMode(BRAKE_RUN,OUTPUT);
    digitalWrite(BRAKE_RUN,LOW);
    pinMode(BRAKE_LOCK,OUTPUT);
    digitalWrite(BRAKE_LOCK,LOW);
    pinMode(BRAKE_RELEASE,OUTPUT);
    digitalWrite(BRAKE_RELEASE,LOW);
    
    eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));                   // These are the configuration settings.
    if(settings.run_duration>MAX_DURATION) settings.run_duration=DEFAULT_DURATION;     // Is greater than max_duration, the configuration has not been set.
    if(settings.state!=LOCK && settings.state!=RELEASE) settings.state=RELEASE;        // If unknown, set the status to RELEASE
    
    delay(DEBOUNCE);
    if(digitalRead(SW_RELEASE)==LOW) {                                                 // If the brake release button is pressed while powering up, setup_mode needs to be entered.
        while(digitalRead(SW_RELEASE)==LOW) {                                          // Wait until the brake release button has been released. Flash the LED to indicate that we've entered setup mode.
            delay(LED_SETUP);
            led(!digitalRead(LED_HI));                                                 // Toggle the LED state
        }
        setup_mode=true;                                                               // Button has been released, now select setup_mode.
    }
    
    led(LOW);                                                                          // Default LED state is off (brake released)
    if(settings.state==LOCK) {
        led(HIGH);                                                                     // but switch the LED on if the last recorded brake state was set to locked.
        if(digitalRead(INDICATION_MODE)==LOW) led_timer=millis()+LED_LONG;             // Set timer to switch off LED if INDICATION_MODE is set to enhanced.
    }
}


void loop() {
    // update the switchstate according to the debounced digital inputs.
    probeswitch(SW_LOCK);
    probeswitch(SW_RELEASE);
    
    // If the engine is switched on, the brake is locked, and the brakemotor is not yet busy unlocking, release the brake NOW!
    if(digitalRead(ENGINE)==HIGH) {
        if(settings.state==LOCK && brakemotorrunning!=RELEASE && engine_released==false) {
            changebrake(RELEASE);
            switchstate=0;                                                             // override switch state.
            engine_released=true;                                                      // Set a flag that we have already released the brake once...
        }                                                                              // ...because in override mode, the brake can be locked again without being automatically released again.
        setup_mode=false;                                                              // Switching on the engine is one way to switch setup_mode off.
    } else {
        engine_released=false;
    }

    // this takes care of changing the time based LED status
    if(brakemotorrunning) {
        if(led_timer<millis()) {
            led(!digitalRead(LED_HI));
            led_timer=millis()+LED_FLASH;
        }
    } else {
        if(digitalRead(INDICATION_MODE)==LOW && switchstate!=0 && switchchanged+SWITCH_SHOW>millis()) {
            led(HIGH);                                                                 // Enhanced indication mode, so we are going to switch on the LED...
            if(settings.state==LOCK) {
                led_timer=millis()+LED_LONG;                                           // ...and have it light up long if the brake is locked...
            } else {
                led_timer=millis()+LED_SHORT;                                          // ...but only short when the brake is released
            }
        }
    
        if(led_timer<millis()) {                                                       // If LED timer has been expired, turn the LED off.
            led(LOW);
        }
    }
    
    if(brakemotorrunning) {                                                            // The brake motor is running. We need to check if we have to stop the brake motor.
        if(setup_mode==false) {                                                        // If we are not in setup mode we will stop the brake motor based on time and current.
            if(!(digitalRead(OVERRIDE_MODE)==LOW && switchstate!=0)) {                 // But only if we are not in override mode while the button is still being pressed
                if(motorstop<millis()) {
                    stopbrake();                                                       // Stop the brake motor because the maximum run time has been expired.
                }
                if(motorstarted+CURRENTDELAY<millis() && analogRead(CURRENT)>settings.max_current) {
                    stopbrake();                                                       // Stop the brake motor because the maximum current has been exceeded.
                }
            }
        } else {
            if(switchstate==0) {
                // We are in setup mode, the brake motor is running, the switches have been released. Now record the new values.
                if(brakemotorrunning==RELEASE) {                                       // In the release direction, we determine the run time of the motor.
                    settings.run_duration=millis()-motorstarted;
                    if(settings.run_duration>MAX_DURATION) settings.run_duration=MAX_DURATION;
                    if(settings.run_duration<MIN_DURATION) settings.run_duration=MIN_DURATION;
                }
                if(brakemotorrunning==LOCK) {                                          // In the lock direction, we determine the maximum current when the brake locks up.
                    settings.max_current=analogRead(CURRENT);
                }
                stopbrake();                                                           // Stopping the brake will also write the new settings to the EEPROM.
            }
        }
    } else {
        if(switchstate==SW_RELEASE && (settings.state==LOCK || digitalRead(OVERRIDE_MODE)==LOW) && !(digitalRead(INDICATION_MODE)==LOW && switchchanged+SWITCH_SHOW>millis())) {
            changebrake(RELEASE);                                                      // Releasing the brake has been requested, so let's do it!
        }
        if(switchstate==SW_LOCK && (settings.state==RELEASE || digitalRead(OVERRIDE_MODE)==LOW) && !(digitalRead(INDICATION_MODE)==LOW && switchchanged+SWITCH_SHOW>millis()) && digitalRead(ENGINE)==LOW) {
            changebrake(LOCK);                                                           // Locking the brake has been requested, so let's do it!
        }
    }
#ifdef LowPower_h    
    // See if we can go to sleep
    if(brakemotorrunning==0 && led_timer<millis() && debounce==0) {
        led(LOW);
        attachInterrupt(0, wakeUp, LOW);
        attachInterrupt(1, wakeUp, LOW);
        LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
        detachInterrupt(1);
        detachInterrupt(0); 
    }
#endif
}


#ifdef LowPower_h
void wakeUp() {
}
#endif


void changebrake(byte bdirection) {
    if(brakemotorrunning) {
        stopbrake();                                                                   // If the brake is still running for some reason, switch it off
        delay(TRANSITIONDELAY);                                                        // Give it a bit time to spin down
    }
    if(bdirection==LOCK) {
        digitalWrite(BRAKE_LOCK,HIGH);                                                 // Turn on the brake motor in LOCK direction
        motorstop=millis()+settings.run_duration+EXTRA_LOCK_DURATION;
    }
    if(bdirection==RELEASE) {
        digitalWrite(BRAKE_RELEASE,HIGH);                                              // Turn on the brake motor in RELEASE direction
        motorstop=millis()+settings.run_duration;
    }
    led(HIGH);
    led_timer=millis()+LED_FLASH;                                                      // Flash the LED to indicate that the brake motor is running
    delay(TRANSITIONDELAY);
    digitalWrite(BRAKE_RUN,HIGH);                                                      // Switch on the brake master power (not used by us, but maybe someone else will?)
    motorstarted=millis();
    brakemotorrunning=bdirection;
}


void stopbrake(void) {
    digitalWrite(BRAKE_RUN,LOW);
    digitalWrite(BRAKE_RELEASE,LOW);
    digitalWrite(BRAKE_LOCK,LOW);
    if(brakemotorrunning) {                                                            // Was the brake motor actually running?
        settings.state=brakemotorrunning;
        eeprom_write_block((void*)&settings, (void*)0, sizeof(settings));              // Record the new state in the EEPROM
        led_timer=0;                                                                   // Reset LED timer
        if(brakemotorrunning==LOCK) {
            led(HIGH);                                                                 // Turn on the LED if we just locked the brake
            if(digitalRead(INDICATION_MODE)==LOW) led_timer=millis()+LED_LONG;         // Set a long timer if we are in enhanced indication mode
        } else {
            if(digitalRead(INDICATION_MODE)==LOW) {
                led(HIGH);                                                             // Only in enhanced indication mode we will light the LED...
                led_timer=millis()+LED_SHORT;                                          // ...but only shortly.
            } else {
                led(LOW);                                                              // Otherwise we'll turn it off
            }
        }
        brakemotorrunning=0;
    }
}


void led(boolean state) {
    digitalWrite(LED_HI,state);                                                        // This is the default LED output
    digitalWrite(LED_LO,!state);                                                       // But this output is for folks who want to have a negative output for some reason. Plenty of ports anyway.
}


void probeswitch(int port) {
    if(switchstate==port) {
        if(probeport(port,HIGH)) {
            switchstate=0;
        }
    } else {
        if(probeport(port,LOW)) {
            switchstate=port;
            switchchanged=millis();
        }
    }
}


// The purpose of this routine is to deal with bouncing switches. Many switches deliver a series of pulses before they settle. So we will have to wait for the switch to become stable.
boolean probeport(int port,boolean state) {
    if(digitalRead(port)==state) {                                                     // Change of state detected!
        if(debounce==0) {                                                              // First time?
            debounce=millis()+DEBOUNCE;                                                // Set timer somewhat in the future
        } else {                                                                       // This is not the first time we detect the change of state
            if(debounce<millis()) {                                                    // Has the switch stopped bouncing?
                debounce=0;                                                            // Reset the timer...
                return HIGH;                                                           // ...and report the change of state.
            }
        }
    }
    return LOW;
}
