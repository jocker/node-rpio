/*
 * Copyright (c) 2015 Jonathan Perkin <jonathan@perkin.org.uk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <nan.h>
#include <unistd.h>	/* usleep() */
#include "bcm2835.h"
#include <chrono>

#define RPIO_EVENT_LOW	0x1
#define RPIO_EVENT_HIGH	0x2



using namespace std;
using namespace Nan;
using namespace v8;



/*
 * GPIO function select.  Pass through all values supported by bcm2835.
 */
NAN_METHOD(gpio_function)
{
	if ((info.Length() != 2) ||
	    !info[0]->IsNumber() ||
	    !info[1]->IsNumber() ||
	    (info[1]->NumberValue() > 7))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_gpio_fsel(info[0]->NumberValue(), info[1]->NumberValue());
}

/*
 * GPIO read/write
 */
NAN_METHOD(gpio_read)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	info.GetReturnValue().Set(bcm2835_gpio_lev(info[0]->NumberValue()));
}

NAN_METHOD(gpio_readbuf)
{
	uint32_t i;
	char *buf;

	if ((info.Length() != 3) ||
	    !info[0]->IsNumber() ||
	    !info[1]->IsObject() ||
	    !info[2]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	buf = node::Buffer::Data(info[1]->ToObject());

	for (i = 0; i < info[2]->NumberValue(); i++)
		buf[i] = bcm2835_gpio_lev(info[0]->NumberValue());
}

NAN_METHOD(gpio_write)
{
	if ((info.Length() != 2) ||
	    !info[0]->IsNumber() ||
	    !info[1]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	bcm2835_gpio_write(info[0]->NumberValue(), info[1]->NumberValue());
}

NAN_METHOD(gpio_writebuf)
{
	uint32_t i;
	char *buf;

	if ((info.Length() != 3) ||
	    !info[0]->IsNumber() ||
	    !info[1]->IsObject() ||
	    !info[2]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	buf = node::Buffer::Data(info[1]->ToObject());

	for (i = 0; i < info[2]->NumberValue(); i++)
		bcm2835_gpio_write(info[0]->NumberValue(), buf[i]);
}

NAN_METHOD(gpio_pad_read)
{
	if ((info.Length() != 1) || !info[0]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	info.GetReturnValue().Set(bcm2835_gpio_pad(info[0]->NumberValue()));
}

NAN_METHOD(gpio_pad_write)
{
	if ((info.Length() != 2) ||
	    !info[0]->IsNumber() ||
	    !info[1]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	bcm2835_gpio_set_pad(info[0]->NumberValue(), info[1]->NumberValue());
}

NAN_METHOD(gpio_pud)
{
	if ((info.Length() != 2) ||
	    !info[0]->IsNumber() ||
	    !info[1]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	/*
	 * We use our own version of bcm2835_gpio_set_pud as that uses
	 * delayMicroseconds() which requires access to the timers and
	 * therefore /dev/mem and root.  Our version is identical, except for
	 * using usleep() instead.
	 */
	bcm2835_gpio_pud(info[1]->NumberValue());
	usleep(10);
	bcm2835_gpio_pudclk(info[0]->NumberValue(), 1);
	usleep(10);
	bcm2835_gpio_pud(BCM2835_GPIO_PUD_OFF);
	bcm2835_gpio_pudclk(info[0]->NumberValue(), 0);
}

NAN_METHOD(gpio_event_set)
{
	if ((info.Length() != 2) ||
	    !info[0]->IsNumber() ||
	    !info[1]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	/* Clear all possible trigger events. */
	bcm2835_gpio_clr_ren(info[0]->NumberValue());
	bcm2835_gpio_clr_fen(info[0]->NumberValue());
	bcm2835_gpio_clr_hen(info[0]->NumberValue());
	bcm2835_gpio_clr_len(info[0]->NumberValue());
	bcm2835_gpio_clr_aren(info[0]->NumberValue());
	bcm2835_gpio_clr_afen(info[0]->NumberValue());

	/*
	 * Add the requested events, using the synchronous rising and
	 * falling edge detection bits.
	 */
	if ((uint32_t)info[1]->NumberValue() & RPIO_EVENT_HIGH)
		bcm2835_gpio_ren(info[0]->NumberValue());

	if ((uint32_t)info[1]->NumberValue() & RPIO_EVENT_LOW)
		bcm2835_gpio_fen(info[0]->NumberValue());
}

NAN_METHOD(gpio_event_poll)
{
	uint32_t rval = 0;

	if ((info.Length() != 1) || !info[0]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	/*
	 * Interrupts are not supported, so this merely reports that an event
	 * happened in the time period since the last poll.  There is no way to
	 * know which trigger caused the event.
	 */
	if ((rval = bcm2835_gpio_eds_multi(info[0]->NumberValue())))
		bcm2835_gpio_set_eds_multi(rval);

	info.GetReturnValue().Set(rval);
}

NAN_METHOD(gpio_event_clear)
{
	if ((info.Length() != 1) || !info[0]->IsNumber())
		return ThrowTypeError("Incorrect arguments");

	bcm2835_gpio_clr_fen(info[0]->NumberValue());
	bcm2835_gpio_clr_ren(info[0]->NumberValue());
}

/*
 * i2c setup
 */
NAN_METHOD(i2c_begin)
{
	bcm2835_i2c_begin();
}

NAN_METHOD(i2c_set_clock_divider)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_i2c_setClockDivider(info[0]->NumberValue());
}

NAN_METHOD(i2c_set_baudrate)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_i2c_set_baudrate(info[0]->NumberValue());
}

NAN_METHOD(i2c_set_slave_address)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_i2c_setSlaveAddress(info[0]->NumberValue());
}

NAN_METHOD(i2c_end)
{
	bcm2835_i2c_end();
}


/*
 * i2c read/write.  The underlying bcm2835_i2c_read/bcm2835_i2c_write functions
 * do not return the number of bytes read/written, only a status code.  The JS
 * layer handles ensuring that the buffer is large enough to accommodate the
 * requested length.
 */
NAN_METHOD(i2c_read)
{
	uint8_t rval;

	if ((info.Length() != 2) ||
	    (!info[0]->IsObject()) ||
	    (!info[1]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	rval = bcm2835_i2c_read(node::Buffer::Data(info[0]->ToObject()),
				info[1]->NumberValue());

	info.GetReturnValue().Set(rval);
}

NAN_METHOD(i2c_write)
{
	uint8_t rval;

	if ((info.Length() != 2) ||
	    (!info[0]->IsObject()) ||
	    (!info[1]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	rval = bcm2835_i2c_write(node::Buffer::Data(info[0]->ToObject()),
				 info[1]->NumberValue());

	info.GetReturnValue().Set(rval);
}

/*
 * PWM functions
 */
NAN_METHOD(pwm_set_clock)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_pwm_set_clock(info[0]->NumberValue());
}

NAN_METHOD(pwm_set_mode)
{
	if ((info.Length() != 3) ||
	    (!info[0]->IsNumber()) ||
	    (!info[1]->IsNumber()) ||
	    (!info[2]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_pwm_set_mode(info[0]->NumberValue(), info[1]->NumberValue(),
			     info[2]->NumberValue());
}

NAN_METHOD(pwm_set_range)
{
	if ((info.Length() != 2) ||
	    (!info[0]->IsNumber()) ||
	    (!info[1]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_pwm_set_range(info[0]->NumberValue(), info[1]->NumberValue());
}

NAN_METHOD(pwm_set_data)
{
	if ((info.Length() != 2) ||
	    (!info[0]->IsNumber()) ||
	    (!info[1]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_pwm_set_data(info[0]->NumberValue(), info[1]->NumberValue());
}

/*
 * SPI functions.
 */
NAN_METHOD(spi_begin)
{
	bcm2835_spi_begin();
}

NAN_METHOD(spi_chip_select)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_spi_chipSelect(info[0]->NumberValue());
}

NAN_METHOD(spi_set_cs_polarity)
{
	if ((info.Length() != 2) ||
	    (!info[0]->IsNumber()) ||
	    (!info[1]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_spi_setChipSelectPolarity(info[0]->NumberValue(),
					  info[1]->NumberValue());
}

NAN_METHOD(spi_set_clock_divider)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_spi_setClockDivider(info[0]->NumberValue());
}

NAN_METHOD(spi_set_data_mode)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_spi_setDataMode(info[0]->NumberValue());
}

NAN_METHOD(spi_transfer)
{
	if ((info.Length() != 3) ||
	    (!info[0]->IsObject()) ||
	    (!info[1]->IsObject()) ||
	    (!info[2]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_spi_transfernb(node::Buffer::Data(info[0]->ToObject()),
			       node::Buffer::Data(info[1]->ToObject()),
			       info[2]->NumberValue());
}

NAN_METHOD(spi_write)
{
	if ((info.Length() != 2) ||
	    (!info[0]->IsObject()) ||
	    (!info[1]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	bcm2835_spi_writenb(node::Buffer::Data(info[0]->ToObject()),
			    info[1]->NumberValue());
}

NAN_METHOD(spi_end)
{
	bcm2835_spi_end();
}

/*
 * Initialize the bcm2835 interface and check we have permission to access it.
 */
NAN_METHOD(rpio_init)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	if (!bcm2835_init(info[0]->NumberValue()))
		return ThrowError("Could not initialize bcm2835 GPIO library");
}

NAN_METHOD(rpio_close)
{
	bcm2835_close();
}

/*
 * Misc functions useful for simplicity
 */
NAN_METHOD(rpio_usleep)
{
	if ((info.Length() != 1) || (!info[0]->IsNumber()))
		return ThrowTypeError("Incorrect arguments");

	usleep(info[0]->NumberValue());
}


//NEW STUFF COMES HERE

int mcp3008_read_adc(uint8_t chan){
	char tx[] = {0x01,(0x08|chan)<<4,0x01};
	char rx[3];
	bcm2835_spi_transfernb(tx,rx,3);
	return ((int)rx[1] & 0x03) << 8 | (int) rx[2];
}

void mcp3008_sample_adc(uint8_t channel, int millis, vector<int> & dest){
	typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::duration<float, std::milli> duration;
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	bcm2835_spi_begin();

	clock::time_point start = clock::now();

	while(true){

        duration elapsed = clock::now() - start;
        if(elapsed.count() > millis){
            break;
        }
		int value = mcp3008_read_adc(channel);

        dest.push_back(value );
    }

	bcm2835_spi_end();
}


class SampleResult{
  public:
	int min = -1;
	int max = -1;
	double sum = 0;
	int avg = 0;
	int count = 0;
	int amplitude = 0;
};

SampleResult calc_sample_result(vector<int> & samples){
	SampleResult result;
	result.count = samples.size();

	for (const int sample : samples) {
   	 	result.sum += sample;
		if(result.min<0 || result.min > sample){
			result.min = sample;
		}
		if(result.max < 0 || result.max < sample){
			result.max = sample;
		}
  	}

if(result.count > 0){
	result.avg = result.sum/result.count;
	result.amplitude = result.max - result.min;
}

	  return result;
}

void pack_sample_result(v8::Isolate* isolate, v8::Local<v8::Object> & target, SampleResult & result){
  target->Set(String::NewFromUtf8(isolate, "min"), Integer::New(isolate, result.min));
  target->Set(String::NewFromUtf8(isolate, "max"), Integer::New(isolate, result.max));
  target->Set(String::NewFromUtf8(isolate, "avg"), Integer::New(isolate, result.avg));
  target->Set(String::NewFromUtf8(isolate, "count"), Integer::New(isolate, result.count));
  target->Set(String::NewFromUtf8(isolate, "sum"), Number::New(isolate, result.sum));
  target->Set(String::NewFromUtf8(isolate, "amplitude"), Integer::New(isolate, result.amplitude));
}

class Mcp3008SampleWorker : public AsyncWorker {
    public:
    Mcp3008SampleWorker(uint8_t channel, int limit, Callback * callback ) : channel(channel), limit(limit),  AsyncWorker(callback) {
    }

    void Execute() {
        mcp3008_sample_adc(channel, limit, samples);
    }

    void HandleOKCallback () {


v8::Isolate* isolate = v8::Isolate::GetCurrent();

SampleResult sampleResult = calc_sample_result(samples);
Local<Object> packedResult = Object::New(isolate);
      pack_sample_result(isolate, packedResult, sampleResult);

        Local<Value> argv[] = { packedResult };
        callback->Call(1, argv);
    }


    private:
    int limit;
	uint8_t channel;
    vector<int> samples;
};

NAN_METHOD(sampleAdc) {

    uint8_t channel = info[0]->Uint32Value();
	int limit = To<int>(info[1]).FromJust();
    Callback *callback = new Callback(info[2].As<Function>());
    AsyncQueueWorker(new Mcp3008SampleWorker(channel, limit, callback));
}

NAN_MODULE_INIT(setup)
{
	NAN_EXPORT(target, rpio_init);
	NAN_EXPORT(target, rpio_close);
	NAN_EXPORT(target, rpio_usleep);
	NAN_EXPORT(target, gpio_function);
	NAN_EXPORT(target, gpio_read);
	NAN_EXPORT(target, gpio_readbuf);
	NAN_EXPORT(target, gpio_write);
	NAN_EXPORT(target, gpio_writebuf);
	NAN_EXPORT(target, gpio_pad_read);
	NAN_EXPORT(target, gpio_pad_write);
	NAN_EXPORT(target, gpio_pud);
	NAN_EXPORT(target, gpio_event_set);
	NAN_EXPORT(target, gpio_event_poll);
	NAN_EXPORT(target, gpio_event_clear);
	NAN_EXPORT(target, i2c_begin);
	NAN_EXPORT(target, i2c_set_clock_divider);
	NAN_EXPORT(target, i2c_set_baudrate);
	NAN_EXPORT(target, i2c_set_slave_address);
	NAN_EXPORT(target, i2c_end);
	NAN_EXPORT(target, i2c_read);
	NAN_EXPORT(target, i2c_write);
	NAN_EXPORT(target, pwm_set_clock);
	NAN_EXPORT(target, pwm_set_mode);
	NAN_EXPORT(target, pwm_set_range);
	NAN_EXPORT(target, pwm_set_data);
	NAN_EXPORT(target, spi_begin);
	NAN_EXPORT(target, spi_chip_select);
	NAN_EXPORT(target, spi_set_cs_polarity);
	NAN_EXPORT(target, spi_set_clock_divider);
	NAN_EXPORT(target, spi_set_data_mode);
	NAN_EXPORT(target, spi_transfer);
	NAN_EXPORT(target, spi_write);
	NAN_EXPORT(target, spi_end);
	NAN_EXPORT(target, sampleAdc);
}

NODE_MODULE(rpio, setup);

