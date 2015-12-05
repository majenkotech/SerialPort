/*
 * Copyright (c) 2015, Majenko Technologies
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "SerialPort.h"

extern "C" int tcdrain (int __fd);

SerialPort::SerialPort(const char *device) {
	_dev = device;
	_fd = -1;
}

bool SerialPort::open(unsigned long baud) {
	struct termios2 options;
	if (_fd >= 0) {
		close();
	}
	_fd = ::open(_dev, O_RDWR|O_NOCTTY);
	if (_fd < 0) {
		return false;
	}
	fcntl(_fd, F_SETFL, 0);

	ioctl(_fd, TCGETS2, &options);
	ioctl(_fd, TCGETS2, &_savedOptions);
	options.c_cflag &= ~CBAUD;
	options.c_cflag |= BOTHER;
	options.c_ispeed = baud;
	options.c_ospeed = baud;
	_savedOptions.c_cflag |= BOTHER;
	_savedOptions.c_ispeed = baud;
	_savedOptions.c_ospeed = baud;
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~CRTSCTS;
	options.c_cflag &= ~HUPCL;
	_savedOptions.c_cflag &= ~HUPCL;

	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
				| INLCR | IGNCR | ICRNL | IXON);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	if (strcmp(_dev, "/dev/tty") == 0) {
		options.c_lflag |= ISIG;
	}

	ioctl(_fd, TCSETS2, &options);
	return true;
}


void SerialPort::close() {
	if (_fd > -1) {
		tcdrain(_fd);
		ioctl(_fd, TCSETS2, &_savedOptions);
		::close(_fd);
	}
	_fd = -1;
}

int SerialPort::available(void) {
	if (_fd < 0) {
		return 0;
	}

	fd_set rfds;
	struct timeval tv;

	FD_ZERO(&rfds);
	FD_SET(_fd, &rfds);
	tv.tv_sec = 0;
	tv.tv_usec = 1;
	int retval = select(_fd+1, &rfds, NULL, NULL, &tv);
	if (retval) {
		return 1;
	}
	return 0;
}

int SerialPort::read(void) {
	if (!available()) {
		return -1;
	}
	uint8_t c;
	if (::read(_fd, &c, 1) <= 0) {
		return -1;
	}
	return c;
}

void SerialPort::write(uint8_t c) {
	if (_fd < 0) {
		return;
	}
	::write(_fd, &c, 1);
}

void SerialPort::write(uint8_t *c, int n) {
	if (_fd < 0) {
		return;
	}
	::write(_fd, c, n);
}

