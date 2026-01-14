// VectorNav SDK Teensy 4.1 HAL Implementation
// Based on VectorNav SDK (v1.0.0)
// 
// The MIT License (MIT)
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef VN_SERIAL_TEENSY_HPP_
#define VN_SERIAL_TEENSY_HPP_

#include <Arduino.h>
#include "vectornav/Config.hpp"
#include "vectornav/HAL/Serial_Base.hpp"
#include "vectornav/TemplateLibrary/String.hpp"

namespace VN
{

class Serial : public Serial_Base
{
public:
    using Serial_Base::Serial_Base;

    // ***********
    // Port access
    // ***********
    Error open(const PortName& portName, const uint32_t baudRate) noexcept override final;
    void close() noexcept override;
    bool isSupportedBaudRate(const uint32_t baudRate) const noexcept override;
    Error changeBaudRate(const uint32_t baudRate) noexcept override final;

    // ***************
    // Port read/write
    // ***************
    Error getData() noexcept override final;
    Error send(const char* buffer, const size_t len) noexcept override final;

private:
    HardwareSerial* _hwSerial = nullptr;
    
    /// @brief Maps port name string to Teensy HardwareSerial pointer
    HardwareSerial* _getSerialPort(const PortName& portName) noexcept;
    
    /// @brief Flush/clear the serial buffers
    void _flush() noexcept;
};

// ######################
//     Implementation
// ######################

inline HardwareSerial* Serial::_getSerialPort(const PortName& portName) noexcept
{
    // Map port names to Teensy 4.1 hardware serial ports
    if (portName == "Serial1") { return &Serial1; }
    else if (portName == "Serial2") { return &Serial2; }
    else if (portName == "Serial3") { return &Serial3; }
    else if (portName == "Serial4") { return &Serial4; }
    else if (portName == "Serial5") { return &Serial5; }
    else if (portName == "Serial6") { return &Serial6; }
    else if (portName == "Serial7") { return &Serial7; }
    else if (portName == "Serial8") { return &Serial8; }
    
    // Also support alternative naming conventions
    else if (portName == "UART1") { return &Serial1; }
    else if (portName == "UART2") { return &Serial2; }
    else if (portName == "UART3") { return &Serial3; }
    else if (portName == "UART4") { return &Serial4; }
    else if (portName == "UART5") { return &Serial5; }
    else if (portName == "UART6") { return &Serial6; }
    else if (portName == "UART7") { return &Serial7; }
    else if (portName == "UART8") { return &Serial8; }
    
    return nullptr;
}

inline Error Serial::open(const PortName& portName, const uint32_t baudRate) noexcept
{
    if (_isOpen) { close(); }

    _hwSerial = _getSerialPort(portName);
    
    if (_hwSerial == nullptr)
    {
        return Error::InvalidPortName;
    }

    // Teensy supports a wide range of baud rates
    // Standard rates work: 9600, 19200, 38400, 57600, 115200, 230400, etc.
    _hwSerial->begin(baudRate);
    
    // Give the UART hardware time to initialize
    // Teensy's begin() is non-blocking, but hardware needs settling time
    delay(10);
    
    // Clear any residual data in buffers
    _flush();

    _portName = portName;
    _baudRate = baudRate;
    _isOpen = true;
    
    return Error::None;
}

inline void Serial::close() noexcept
{
    if (_hwSerial != nullptr && _isOpen)
    {
        _flush();
        _hwSerial->end();
        _hwSerial = nullptr;
    }
    _isOpen = false;
}

inline bool Serial::isSupportedBaudRate(const uint32_t baudRate) const noexcept
{
    // Teensy 4.1 supports a very wide range of baud rates
    // The UART hardware can handle from ~300 to ~12,000,000 baud
    // Common rates: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
    // For VectorNav sensors, typical range is 9600 to 921600
    
    // Set reasonable bounds
    return (baudRate >= 300 && baudRate <= 2000000);
}

inline Error Serial::changeBaudRate(const uint32_t baudRate) noexcept
{
    if (!_isOpen) { return Error::SerialPortClosed; }
    if (_hwSerial == nullptr) { return Error::SerialPortClosed; }
    
    // Flush buffers before changing baud rate
    _flush();
    
    // Teensy requires end() before changing baud rate
    _hwSerial->end();
    delay(10);
    
    // Reinitialize with new baud rate
    _hwSerial->begin(baudRate);
    delay(10);
    
    // Clear any noise/garbage from baud rate transition
    _flush();
    
    _baudRate = baudRate;
    
    return Error::None;
}

inline Error Serial::getData() noexcept
{
    if (!_isOpen) { return Error::SerialPortClosed; }
    if (_hwSerial == nullptr) { return Error::SerialPortClosed; }

    // Check how many bytes are available in the hardware buffer
    int bytesAvailable = _hwSerial->available();
    
    if (bytesAvailable <= 0)
    {
        return Error::None;  // No data available, not an error
    }

    // Read data into the byte buffer
    size_t linearBytes = _byteBuffer.numLinearBytesToPut();
    
    while (bytesAvailable > 0 && linearBytes > 0)
    {
        // Read as many bytes as we can fit in one linear chunk
        size_t bytesToRead = (bytesAvailable < linearBytes) ? bytesAvailable : linearBytes;
        
        // Read directly into the buffer's tail pointer
        size_t bytesRead = _hwSerial->readBytes(
            const_cast<uint8_t*>(_byteBuffer.tail()), 
            bytesToRead
        );
        
        if (bytesRead == 0)
        {
            // Timeout or read error
            return Error::SerialReadFailed;
        }
        
        // Update the buffer's put position
        _byteBuffer.put(bytesRead);
        
        // Update remaining counts
        bytesAvailable -= bytesRead;
        linearBytes = _byteBuffer.numLinearBytesToPut();
    }

    // If there are still bytes available but no linear space in buffer
    if (bytesAvailable > 0)
    {
        return Error::PrimaryBufferFull;
    }

    return Error::None;
}

inline Error Serial::send(const char* buffer, const size_t len) noexcept
{
    if (!_isOpen) { return Error::SerialPortClosed; }
    if (_hwSerial == nullptr) { return Error::SerialPortClosed; }
    
    size_t bytesWritten = _hwSerial->write(reinterpret_cast<const uint8_t*>(buffer), len);
    
    if (bytesWritten != len)
    {
        return Error::SerialWriteFailed;
    }
    
    // Teensy automatically handles transmission
    // No need to manually wait, but you can call flush() if needed:
    // _hwSerial->flush(); // This waits for transmission to complete
    
    return Error::None;
}

inline void Serial::_flush() noexcept
{
    if (_hwSerial != nullptr)
    {
        // Wait for outgoing data to be transmitted
        _hwSerial->flush();
        
        // Clear incoming buffer
        while (_hwSerial->available() > 0)
        {
            _hwSerial->read();
        }
    }
}

}  // namespace VN

#endif  // VN_SERIAL_TEENSY_HPP_