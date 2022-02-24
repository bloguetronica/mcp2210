/* Extra functions for libusb - Version 1.0.3
   Copyright (c) 2018-2021 Samuel Lourenço

   This library is free software: you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or (at your
   option) any later version.

   This library is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this library.  If not, see <https://www.gnu.org/licenses/>.


   Please feel free to contact me via e-mail: samuel.fmlourenco@gmail.com */


#ifndef LIBUSB_EXTRA_H_
#define LIBUSB_EXTRA_H_

// Includes
#include <libusb-1.0/libusb.h>

// Function prototypes
libusb_device_handle *libusb_open_device_with_vid_pid_serial(libusb_context *context, uint16_t vid, uint16_t pid, unsigned char *serial);

#endif
