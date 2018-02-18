#include "dht11.h"

static struct dht11_operations *dht11_ops;
static uint32_t expectPulse(uint8_t pin_level);

uint8_t DHT11_getData(struct dht11_operations *dht11_tops, dht11_data_t *dht11_data) {
    
    uint32_t cycles[80];
    uint8_t data[5];
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;
    
    //start condition
    dht11_ops = dht11_tops;
    dht11_ops->pinWrite(0);
	dht11_ops->delay_ms(20);
    dht11_ops->pinWrite(1);
    
    //wait response from sensor
    dht11_ops->delay_us(30);   
    if(expectPulse(0) == 0) { //error timeout
        //return -1;
    }
    if(expectPulse(1) == 0) { //error timeout
        //return -1;
    }
    
    //read data from sensor
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(0);
      cycles[i+1] = expectPulse(1);
    }
    
    
    for (int i=0; i<40; ++i) {
        uint32_t lowCycles  = cycles[2*i];
        uint32_t highCycles = cycles[2*i+1];
//        if ((lowCycles == 0) || (highCycles == 0)) {
//            _lastresult = false;
//            return _lastresult;
//        }
        data[i/8] <<= 1;
    
        // Now compare the low and high cycle times to see if the bit is a 0 or 1.
        if (highCycles > lowCycles) {
            // High cycles are greater than 50us low cycle count, must be a 1.
            data[i/8] |= 1;
        }
        // Else high cycles are less than (or equal to, a weird case) the 50us low
        // cycle count so this must be a zero.  Nothing needs to be changed in the
        // stored data.
    }
    
    dht11_data->humidity = data[0];
    dht11_data->temper = data[2];  
  
	return DHT11_OK;
}

static uint32_t expectPulse(uint8_t pin_level) {
	uint32_t pulse_tick = 0;

	while (1) {		
        pulse_tick++;
        if (pulse_tick >= 0xFFFF) {
			return 0;
		}
		if (dht11_ops->readPin() != pin_level) {
			return pulse_tick;
		}
	}
}

