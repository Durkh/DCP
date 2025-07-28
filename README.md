The DCP (Devices Communication Protocol) is a one wired bidirectional multimaster low-speed communication protocol. Aimed for distributed embedded systems communication but not limited to.

It uses a bus architecture supporting controller and targets in the same bus, as well as broadcasting and adressed messages.

# Contents

- [The Protocol](#the-protocol)
- [About this library](#this-library)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Documents](#documents)

# The protocol

The protocol works by following a series of rules that asserts a successful communication in the bus:

- CSMA/CS: the device must listen for incoming messages
- binary countdown
- CSMA/CD: bus yielding after high priority collision

It supports more than one controller and more than one peripheral per bus, but it is limited to 255 devices per bus. Each device must have a unique 8-bit busID (hece the 255 device limit) and devices with **lower** busID have priority in sending messages in collisions.

The devices *drive* the bus using open-collector IO, and as such, short circuit condition does not happen, but a pull-up resistor is necessary. For small buses (low capacitance) and low-speed communication, internal pull-ups, which usually have high values (100kΩ) can be used. Although, incorrect choice of pull-up resistor can create communication errors. Strategies similar to I²C can be used.

The protocol support the following speed modes:

|Mode| Speed|
|---|---|
|Slow | Δ = 20μs|
|Fast I | Δ = 4μs|
|Fast II | Δ = 2.5μs|
|Ultra | Δ = 1.25μs|

in which Δ (delta) is the unit of time of time that composes all the signals of the protocol. That being, signals range from 1Δ to 50Δ.

## MAC

To transmit data, the device must attend to the following MAC scheme:

- Start sync, to alert all the devices of a message;
- Bit sync, to synchronize the bit timing of all the devices;
- MAC ID, for binary countdown.

Then, a packet can be sent.

## LLC Packets

The the packets are structured as follows:

| Header| Payload|
|---|---|
| 1 byte| up to 254 bytes|

### Header

The header must be one of the values: 0, signifying that a 12 bytes [L3](https://repositorio.ufpb.br/jspui/handle/123456789/19731?locale=pt_BR) message follows; or the size, in bytes, of the following message.

A generic message can be of any structure you want, but a L3 message **must** use the appropriate structure and helper functions, defined in the [header](include/DCP.h).

# This library

This library is organized as follows:

- generic code
- ports
    -   RP2350
    -   STM32F4
    -   ESP32C3

The generic code contains the application logic of the protocol, including: the main FSM, read interrupt, send and read public functions.

The port codes are responsible for initializing resources used by the library and implement the low-level driver for hardware inteface, such as gpio and hardware timer interfaces.

As the protocol uses only one wire for data and timing, the communication speed must be agreed upon before communication, in similar fashion with UART. 

## Usage

### ESP-IDF

This library can be simply cloned into the `components/` folder and called into code.

### CMake

in your CMakeLists.txt just add it as a library, for example:

```
add_library(DCP OBJECT)
target_sources(DCP PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/DCP/generic/DCP.c
    ${CMAKE_SOURCE_DIR}/Core/DCP/STM32/port.c
)
target_link_libraries(DCP PUBLIC myproject)
```

> Change the directories and project name accordingly. As seen in [dependencies](#depedencies), this must be added after the FreeRTOS library.

## Depedencies

Currently, this library depends on:

- FreeRTOS (Task and Queue);
- hardware timer, for bit timing;
- cmake, for drop-in integration with supported architectures.

# Documents

An overview of the protocol and applications examples can be found [here](docs/manual.pdf)
