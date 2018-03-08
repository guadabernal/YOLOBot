#ifndef SERIALPORT_H_INCLUDED
#define SERIALPORT_H_INCLUDED

#define ARDUINO_WAIT_TIME 2000

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

class SerialPort {
public:

	SerialPort(char *portName = nullptr) : connected(false) {
		if (portName)
			connect(portName);
	}

	void connect(char* portName) {
		wchar_t wtext[20];
		mbstowcs(wtext, portName, strlen(portName) + 1);//Plus null
		LPWSTR ptr = wtext;
		hSerial = CreateFileW(ptr, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		if (hSerial == INVALID_HANDLE_VALUE) {
			if (GetLastError() == ERROR_FILE_NOT_FOUND)
				printf("ERROR: Handle was not attached. %s not available.\n", portName);
			else
				printf("ERROR!!!\n");
		}
		else {
			DCB dcbSerialParams = { 0 };
			if (!GetCommState(hSerial, &dcbSerialParams)) {
				printf("failed to get current serial parameters!\n");
			}
			else {
				dcbSerialParams.BaudRate = CBR_57600;
				dcbSerialParams.ByteSize = 8;
				dcbSerialParams.StopBits = ONESTOPBIT;
				dcbSerialParams.Parity = NOPARITY;
				// Setting the DTR to Control_Enable ensures that the Arduino is
				// properly reset upon establishing a connection
				//  dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
				if (!SetCommState(hSerial, &dcbSerialParams)) {
					printf("Error: Could not set Serial Port parameters\n");
				}
				else {
					printf("Connected to port %s\n", portName);
					connected = true;
					PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
					Sleep(ARDUINO_WAIT_TIME);
				}
			}
		}
	}

	~SerialPort() {
		if (connected) {
			connected = false;
			CloseHandle(hSerial);
		}
	}

	size_t ReadData(char *buffer, unsigned int nbChar) {
		DWORD bytesRead = 0;
		ClearCommError(hSerial, &errors, &status);
		if (status.cbInQue > 0) {
			size_t toRead = status.cbInQue > nbChar ? nbChar : status.cbInQue;
			ReadFile(hSerial, buffer, toRead, &bytesRead, NULL);
		}
		return bytesRead;
	}

	size_t WriteData(char *buffer, size_t nbChar) {
		DWORD bytesSend = 0;
		if (!WriteFile(hSerial, (void *)buffer, nbChar, &bytesSend, 0))
			ClearCommError(hSerial, &errors, &status);
		return bytesSend;
	}

	bool IsConnected() { return connected; }

private:
	HANDLE hSerial;
	bool connected;
	COMSTAT status;
	DWORD errors;
};

#endif // SERIALPORT_H_INCLUDED