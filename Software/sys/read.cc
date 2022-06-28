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
    sys_read(int fd, void *buf, size_t sz, int &nread) {
        nread = 0;
        if (fd > 2) {
            if (getBasicChipsetInterface().readFile(fd - 3, buf, sz, nread)) {
                return 0;
            } else {
                return EBADF;
            }
        } else {
            // builtin files
            switch (fd) {
                case STDIN_FILENO:
                    nread = getBasicChipsetInterface().read(reinterpret_cast<char *>(buf), sz);
                    return 0;
                default:
                    return EBADF;
            }
        }
    }
}

extern "C"
int
read (int fd, void* buf, size_t sz) {
    //printf("read(%d, 0x%x, %ld)\n", fd, buf, sz);
    int nread = 0;
    int r = sys_read (fd, buf, sz, nread);
    if (r != 0)
    {
        errno = r;
        return -1;
    }
    return nread;
}
