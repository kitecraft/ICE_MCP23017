#include "Ice_MCP23017.h"

Ice_MCP23017::Ice_MCP23017(uint8_t address, TwoWire& bus) {
	_deviceAddr = address;
	_bus = &bus;
}

Ice_MCP23017::~Ice_MCP23017() {}

void Ice_MCP23017::init()
{
	//BANK = 	0 : sequential register addresses
	//MIRROR = 	0 : use configureInterrupt
	//SEQOP = 	1 : sequential operation disabled, address pointer does not increment
	//DISSLW = 	0 : slew rate enabled
	//HAEN = 	0 : hardware address pin is always enabled on 23017
	//ODR = 	0 : open drain output
	//INTPOL = 	0 : interrupt active low
	writeRegister(Ice_MCP23017Register::IOCON, 0b00100000);

	//enable all pull up resistors (will be effective for input pins only)
	//writeRegister(Ice_MCP23017Register::GPPU_A, 0xFF, 0xFF);
}

void Ice_MCP23017::portMode(Ice_MCP23017Port port, uint8_t directions, uint8_t pullups, uint8_t inverted)
{
	writeRegister(Ice_MCP23017Register::IODIR_A + port, directions);
	writeRegister(Ice_MCP23017Register::GPPU_A + port, pullups);
	writeRegister(Ice_MCP23017Register::IPOL_A + port, inverted);
}

void Ice_MCP23017::pinMode(uint8_t pin, uint8_t mode, bool inverted)
{
	Ice_MCP23017Register iodirreg = Ice_MCP23017Register::IODIR_A;
	Ice_MCP23017Register pullupreg = Ice_MCP23017Register::GPPU_A;
	Ice_MCP23017Register polreg = Ice_MCP23017Register::IPOL_A;
	uint8_t iodir, pol, pull;

	if(pin > 7)
	{
		iodirreg = Ice_MCP23017Register::IODIR_B;
		pullupreg = Ice_MCP23017Register::GPPU_B;
		polreg = Ice_MCP23017Register::IPOL_B;
		pin -= 8;
	}

	iodir = readRegister(iodirreg);
	if(mode == INPUT || mode == INPUT_PULLUP) bitSet(iodir, pin);
	else bitClear(iodir, pin);

	pull = readRegister(pullupreg);
	if(mode == INPUT_PULLUP) bitSet(pull, pin);
	else bitClear(pull, pin);

	pol = readRegister(polreg);
	if(inverted) bitSet(pol, pin);
	else bitClear(pol, pin);

	writeRegister(iodirreg, iodir);
	writeRegister(pullupreg, pull);
	writeRegister(polreg, pol);
}

void Ice_MCP23017::digitalWrite(uint8_t pin, uint8_t state)
{
	Ice_MCP23017Register gpioreg = Ice_MCP23017Register::GPIO_A;
	uint8_t gpio;
	if(pin > 7)
	{
		gpioreg = Ice_MCP23017Register::GPIO_B;
		pin -= 8;
	}

	gpio = readRegister(gpioreg);
	if(state == HIGH) bitSet(gpio, pin);
	else bitClear(gpio, pin);

	writeRegister(gpioreg, gpio);
}

uint8_t Ice_MCP23017::digitalRead(uint8_t pin)
{
	Ice_MCP23017Register gpioreg = Ice_MCP23017Register::GPIO_A;
	uint8_t gpio;
	if(pin > 7)
	{
		gpioreg = Ice_MCP23017Register::GPIO_B;
		pin -=8;
	}

	gpio = readRegister(gpioreg);
	if(bitRead(gpio, pin)) return HIGH;
	return LOW;
}

void Ice_MCP23017::writePort(Ice_MCP23017Port port, uint8_t value)
{
	writeRegister(Ice_MCP23017Register::GPIO_A + port, value);
}

void Ice_MCP23017::write(uint16_t value)
{
	writeRegister(Ice_MCP23017Register::GPIO_A, lowByte(value), highByte(value));
}

uint8_t Ice_MCP23017::readPort(Ice_MCP23017Port port)
{
	return readRegister(Ice_MCP23017Register::GPIO_A + port);
}

uint16_t Ice_MCP23017::read()
{
	uint8_t a = readPort(Ice_MCP23017Port::A);
	uint8_t b = readPort(Ice_MCP23017Port::B);

	return a | b << 8;
}

void Ice_MCP23017::writeRegister(Ice_MCP23017Register reg, uint8_t value)
{
	_bus->beginTransmission(_deviceAddr);
	_bus->write(static_cast<uint8_t>(reg));
	_bus->write(value);
	_bus->endTransmission();
}

void Ice_MCP23017::writeRegister(Ice_MCP23017Register reg, uint8_t portA, uint8_t portB)
{
	_bus->beginTransmission(_deviceAddr);
	_bus->write(static_cast<uint8_t>(reg));
	_bus->write(portA);
	_bus->write(portB);
	_bus->endTransmission();
}


uint8_t Ice_MCP23017::readRegister(Ice_MCP23017Register reg)
{
	_bus->beginTransmission(_deviceAddr);
	_bus->write(static_cast<uint8_t>(reg));
	_bus->endTransmission();
	_bus->requestFrom(_deviceAddr, (uint8_t)1);
	return _bus->read();
}


uint8_t Ice_MCP23017::readRegister(uint8_t reg)
{
	_bus->beginTransmission(_deviceAddr);
	_bus->write(static_cast<uint8_t>(reg));
	_bus->endTransmission();
	_bus->requestFrom(_deviceAddr, (uint8_t)1);
	return _bus->read();
}

void Ice_MCP23017::readRegister(Ice_MCP23017Register reg, uint8_t& portA, uint8_t& portB)
{
	_bus->beginTransmission(_deviceAddr);
	_bus->write(static_cast<uint8_t>(reg));
	_bus->endTransmission();
	_bus->requestFrom(_deviceAddr, (uint8_t)2);
	portA = _bus->read();
	portB = _bus->read();
}

#ifdef _MCP23017_INTERRUPT_SUPPORT_

void Ice_MCP23017::interruptMode(Ice_MCP23017InterruptMode intMode)
{
	uint8_t iocon = readRegister(Ice_MCP23017Register::IOCON);
	if(intMode == Ice_MCP23017InterruptMode::Or) iocon |= static_cast<uint8_t>(Ice_MCP23017InterruptMode::Or);
	else iocon &= ~(static_cast<uint8_t>(Ice_MCP23017InterruptMode::Or));

	writeRegister(Ice_MCP23017Register::IOCON, iocon);
}

void Ice_MCP23017::interrupt(Ice_MCP23017Port port, uint8_t mode)
{
	Ice_MCP23017Register defvalreg = Ice_MCP23017Register::DEFVAL_A + port;
	Ice_MCP23017Register intconreg = Ice_MCP23017Register::INTCON_A + port;

	//enable interrupt for port
	writeRegister(Ice_MCP23017Register::GPINTEN_A + port, 0xFF);
	switch(mode)
	{
	case CHANGE:
		//interrupt on change
		writeRegister(intconreg, 0);
		break;
	case FALLING:
		//interrupt falling : compared against defval, 0xff
		writeRegister(intconreg, 0xFF);
		writeRegister(defvalreg, 0xFF);
		break;
	case RISING:
		//interrupt rising : compared against defval, 0x00
		writeRegister(intconreg, 0xFF);
		writeRegister(defvalreg, 0x00);
		break;
	}
}

void Ice_MCP23017::interruptedBy(uint8_t& portA, uint8_t& portB)
{
	readRegister(Ice_MCP23017Register::INTF_A, portA, portB);
}

void Ice_MCP23017::disableInterrupt(Ice_MCP23017Port port)
{
	writeRegister(Ice_MCP23017Register::GPINTEN_A + port, 0x00);
}

void Ice_MCP23017::clearInterrupts()
{
	uint8_t a, b;
	clearInterrupts(a, b);
}

void Ice_MCP23017::clearInterrupts(uint8_t& portA, uint8_t& portB)
{
	readRegister(Ice_MCP23017Register::INTCAP_A, portA, portB);
}


int8_t Ice_MCP23017::getLastInterruptPin()
{
	uint8_t intf;

	// try port A
	intf = readRegister(Ice_MCP23017Register::INTF_A);
	for (int i = 0; i < 8; i++)
	if (bitRead(intf, i))
	  return i;

	// try port B
	intf = readRegister(Ice_MCP23017Register::INTF_B);
	for (int i = 0; i < 8; i++)
	if (bitRead(intf, i))
	  return i + 8;

	return MCP23017_INT_ERR;
}

uint8_t Ice_MCP23017::getLastInterruptPinValue()
{
	uint8_t intPin = getLastInterruptPin();
	if (intPin != MCP23017_INT_ERR) {
	uint8_t intcapreg = regForPin(intPin, (uint8_t)Ice_MCP23017Register::INTCAP_A, (uint8_t)Ice_MCP23017Register::INTCAP_B);
	uint8_t bit = bitForPin(intPin);
	return (readRegister(intcapreg) >> bit) & (0x01);
	}

	return MCP23017_INT_ERR;
}
uint8_t Ice_MCP23017::bitForPin(uint8_t pin)
{
	return pin % 8;
}

uint8_t Ice_MCP23017::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr)
{
	return (pin < 8) ? portAaddr : portBaddr;
}
#endif
