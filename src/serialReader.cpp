//
// Created by hanur on 19.02.2025.
//

#include "serialReader.hpp"


serialReader::serialReader(const std::string port, const int rate, const int timeout)
        : serialPort(port), baudRate(rate), simpleTimeout(timeout),
          mySerial(port, rate, serial::Timeout::simpleTimeout(timeout))
{
}

void serialReader::runSerialReceiver() {}

void serialReader::runSerialSender() {}