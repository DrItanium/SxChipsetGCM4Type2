/*
hitagimon
Copyright (c) 2020-2021, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by jwscoggins on 6/29/21.
//
#include <unistd.h>
#include <errno.h>
#include "chipset/IODevice.h"
namespace
{
    int
    sys_write(int fd, const void *buf, size_t sz, int &nwrite) {
        nwrite = 0;
        if (fd > 2) {
            if (getBasicChipsetInterface().writeFile(fd - 3, buf, sz, nwrite)) {
                return 0;
            } else {
                return EBADF;
            }
        } else {
            // builtin files
            switch (fd) {
                case STDOUT_FILENO:
                case STDERR_FILENO:
                    nwrite = getBasicChipsetInterface().write(reinterpret_cast<char *>(const_cast<void *>(buf)), sz);
                    break;
                default:
                    return EBADF;
            }
            return 0;
        }
    }
}
extern "C"
int
write (int fd, const void* buf, size_t sz) {
    int numWritten = 0;
    int r = sys_write(fd, buf, sz, numWritten);
    if (r != 0) {
        errno = r;
        return -1;
    }
    return numWritten;
}
