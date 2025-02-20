//
// Created by hanur on 19.02.2025.
//

#include "serialReader.hpp"


serialReader::serialReader(const std::string port, const int rate, const int timeout)
        : serialPort(port), baudRate(rate), simpleTimeout(timeout),
          mySerial(port, rate, serial::Timeout::simpleTimeout(timeout))
{
}

void serialReader::runSerialReceiver() {
    if (mySerial.available()) {
        std::string line = mySerial.readline();
        auto values = splitCSV(line);

        if (values.size() == 3) { //can propably delete this line
            try {
                double val1 = std::stod(values[0]);
                double val2 = std::stod(values[1]);
                double val3 = std::stod(values[2]);

            } catch (...) {
                qDebug() << "Error parsing CSV values";
            }
        }
    }
}

void serialReader::runSerialSender() {

}

std::vector<std::string> serialReader::splitCSV(const std::string& line, char delimiter = ',') {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}