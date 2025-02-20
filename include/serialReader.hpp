#include <iostream>
#include <mutex>
#include "serial/serial.h"

#ifndef UNSENTENCEDKALMAN_SERIALREADER_H
#define UNSENTENCEDKALMAN_SERIALREADER_H


class serialReader {
public:
    serialReader(const std::string port, const int rate, const int timeout);
    void runSerialReceiver();
    void runSerialSender();
    std::vector<std::string> serialReader::splitCSV(const std::string& line, char delimiter = ',');
private:
    serial::Serial mySerial;
    std::string serialPort;
    const int baudRate;
    const int simpleTimeout;
    std::tuple<double, double, double, double, double> receivedData;

};



#endif //UNSENTENCEDKALMAN_SERIALREADER_H
