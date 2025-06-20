/*
 * FIFO circular buffer
 *
 * @file        CO_fifo.c
 * @ingroup     CO_CANopen_309_fifo
 * @author      Janez Paternoster
 * @copyright   2020 Janez Paternoster
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#include <string.h>

#include "CO_fifo.h"

#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_ENABLE) != 0

#include <ctype.h>
#include <stdlib.h>


#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_ASCII_COMMANDS) != 0
#include <stdio.h>
#include <inttypes.h>

/* Non-graphical character for command delimiter */
#define DELIM_COMMAND ((uint8_t)'\n')
/* Graphical character for comment delimiter */
#define DELIM_COMMENT ((uint8_t)'#')
/* Graphical character for double quotes */
#define DELIM_DQUOTE  ((uint8_t)'"')
#endif /* (CO_CONFIG_FIFO) & CO_CONFIG_FIFO_ASCII_COMMANDS */

/* verify configuration */
#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_CRC16_CCITT) != 0
#if ((CO_CONFIG_CRC16)&CO_CONFIG_CRC16_ENABLE) == 0
#error CO_CONFIG_CRC16_ENABLE must be enabled.
#endif
#endif

void
CO_fifo_init(CO_fifo_t* fifo, uint8_t* buf, size_t bufSize) {

    if ((fifo == NULL) || (buf == NULL) || (bufSize < 2U)) {
        return;
    }

    fifo->readPtr = 0;
    fifo->writePtr = 0;
    fifo->buf = buf;
    fifo->bufSize = bufSize;

    return;
}

/* Circular FIFO buffer example for fifo->bufSize = 7 (usable size = 6): ******
 *                                                                            *
 *   0      *            *             *            *                         *
 *   1    rp==wp      readPtr      writePtr         *                         *
 *   2      *            *             *            *                         *
 *   3      *            *             *        writePtr                      *
 *   4      *        writePtr       readPtr      readPtr                      *
 *   5      *            *             *            *                         *
 *   6      *            *             *            *                         *
 *                                                                            *
 *        empty       3 bytes       4 bytes       buffer                      *
 *        buffer      in buff       in buff       full                        *
 ******************************************************************************/
size_t
CO_fifo_write(CO_fifo_t* fifo, const uint8_t* buf, size_t count, uint16_t* crc) {
    size_t i;
    uint8_t* bufDest;

    if ((fifo == NULL) || (fifo->buf == NULL) || (buf == NULL)) {
        return 0;
    }

    bufDest = &fifo->buf[fifo->writePtr];
    for (i = count; i > 0U; i--) {
        size_t writePtrNext = fifo->writePtr + 1U;

        /* is circular buffer full */
        if ((writePtrNext == fifo->readPtr) || ((writePtrNext == fifo->bufSize) && (fifo->readPtr == 0U))) {
            break;
        }

        *bufDest = *buf;

#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_CRC16_CCITT) != 0
        if (crc != NULL) {
            crc16_ccitt_single(crc, *buf);
        }
#endif

        /* increment variables */
        if (writePtrNext == fifo->bufSize) {
            fifo->writePtr = 0;
            bufDest = &fifo->buf[0];
        } else {
            fifo->writePtr++;
            bufDest++;
        }
        buf++;
    }

    return count - i;
}

size_t
CO_fifo_read(CO_fifo_t* fifo, uint8_t* buf, size_t count, bool_t* eof) {
    size_t i;
    const uint8_t* bufSrc;
    bool_t alive_cycle = true;

    if (eof != NULL) {
        *eof = false;
    }
    if ((fifo == NULL) || (buf == NULL) || (fifo->readPtr == fifo->writePtr)) {
        return 0;
    }

    bufSrc = &fifo->buf[fifo->readPtr];
    for (i = count; (i > 0U) && alive_cycle;) {
        const uint8_t c = *bufSrc;

        /* is circular buffer empty */
        if (fifo->readPtr == fifo->writePtr) {
            alive_cycle = false;
        } else {
            *buf = c;
            buf++;

            /* increment variables */
            if (++fifo->readPtr == fifo->bufSize) {
                fifo->readPtr = 0;
                bufSrc = &fifo->buf[0];
            } else {
                bufSrc++;
            }
            i--;

#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_ASCII_COMMANDS) != 0
            /* is delimiter? */
            if ((eof != NULL) && (c == DELIM_COMMAND)) {
                *eof = true;
                alive_cycle = false;
            }
#endif
        }
    }

    return count - i;
}

#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_ALT_READ) != 0
size_t
CO_fifo_altBegin(CO_fifo_t* fifo, size_t offset) {
    size_t i;

    if (fifo == NULL) {
        return 0;
    }

    fifo->altReadPtr = fifo->readPtr;
    for (i = offset; i > 0U; i--) {
        /* is circular buffer empty */
        if (fifo->altReadPtr == fifo->writePtr) {
            break;
        }

        /* increment variable */
        if (++fifo->altReadPtr == fifo->bufSize) {
            fifo->altReadPtr = 0;
        }
    }

    return offset - i;
}

void
CO_fifo_altFinish(CO_fifo_t* fifo, uint16_t* crc) {

    if (fifo == NULL) {
        return;
    }

    if (crc == NULL) {
        fifo->readPtr = fifo->altReadPtr;
    } else {
        const uint8_t* bufSrc = &fifo->buf[fifo->readPtr];
        while (fifo->readPtr != fifo->altReadPtr) {
#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_CRC16_CCITT) != 0
            crc16_ccitt_single(crc, *bufSrc);
#endif
            /* increment variable */
            if (++fifo->readPtr == fifo->bufSize) {
                fifo->readPtr = 0;
                bufSrc = &fifo->buf[0];
            } else {
                bufSrc++;
            }
        }
    }
}

size_t
CO_fifo_altRead(CO_fifo_t* fifo, uint8_t* buf, size_t count) {
    size_t i;
    const uint8_t* bufSrc;

    bufSrc = &fifo->buf[fifo->altReadPtr];
    for (i = count; i > 0U; i--) {
        const uint8_t c = *bufSrc;

        /* is there no more data */
        if (fifo->altReadPtr == fifo->writePtr) {
            break;
        }

        *buf = c;
        buf++;

        /* increment variables */
        if (++fifo->altReadPtr == fifo->bufSize) {
            fifo->altReadPtr = 0;
            bufSrc = &fifo->buf[0];
        } else {
            bufSrc++;
        }
    }

    return count - i;
}
#endif /* (CO_CONFIG_FIFO) & CO_CONFIG_FIFO_ALT_READ */

#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_ASCII_COMMANDS) != 0
bool_t
CO_fifo_CommSearch(CO_fifo_t* fifo, bool_t clear) {
    bool_t newCommand = false;
    size_t count;
    uint8_t* commandEnd;

    if ((fifo == NULL) || (fifo->readPtr == fifo->writePtr)) {
        return false;
    }

    /* search delimiter until writePtr or until end of buffer */
    if (fifo->readPtr < fifo->writePtr) {
        count = fifo->writePtr - fifo->readPtr;
    } else {
        count = fifo->bufSize - fifo->readPtr;
    }
    commandEnd = (uint8_t*)memchr((const void*)&fifo->buf[fifo->readPtr], (int32_t)DELIM_COMMAND, count);
    if (commandEnd != NULL) {
        newCommand = true;
    } else if (fifo->readPtr > fifo->writePtr) {
        /* not found, search in the beginning of the circular buffer */
        commandEnd = (uint8_t*)memchr((const void*)&fifo->buf[0], (int32_t)DELIM_COMMAND, fifo->writePtr);
        if ((commandEnd != NULL) || (fifo->readPtr == (fifo->writePtr + 1U))) {
            /* command delimiter found or buffer full */
            newCommand = true;
        }
    } else if ((fifo->readPtr == 0U) && (fifo->writePtr == (fifo->bufSize - 1U))) {
        /* buffer full */
        newCommand = true;
    } else { /* MISRA C 2004 14.10 */
    }

    /* Clear buffer if set so */
    if (clear) {
        if (commandEnd != NULL) {
            fifo->readPtr = ((size_t)commandEnd - (size_t)fifo->buf) + 1U;
            if (fifo->readPtr == fifo->bufSize) {
                fifo->readPtr = 0;
            }
        } else {
            fifo->readPtr = fifo->writePtr;
        }
    }

    return newCommand;
}

bool_t
CO_fifo_trimSpaces(CO_fifo_t* fifo, bool_t* insideComment) {
    bool_t delimCommandFound = false;
    bool_t alive_cycle = true;

    if ((fifo != NULL) && (insideComment != NULL)) {
        while ((fifo->readPtr != fifo->writePtr) && alive_cycle) {
            uint8_t c = fifo->buf[fifo->readPtr];

            if (c == DELIM_COMMENT) {
                *insideComment = true;
            } else if ((isgraph((int)c) != 0) && !(*insideComment)) {
                alive_cycle = false;
            } else { /* MISRA C 2004 14.10 */
            }

            if (alive_cycle) {
                if (++fifo->readPtr == fifo->bufSize) {
                    fifo->readPtr = 0;
                }
                if (c == DELIM_COMMAND) {
                    delimCommandFound = true;
                    *insideComment = false;
                    alive_cycle = false;
                }
            }
        }
    }
    return delimCommandFound;
}

size_t
CO_fifo_readToken(CO_fifo_t* fifo, char* buf, size_t count, uint8_t* closed, bool_t* err) {
    bool_t delimCommandFound = false;
    bool_t delimCommentFound = false;
    size_t tokenSize = 0;

    if ((fifo != NULL) && (buf != NULL) && (count > 1U) && ((err == NULL) || (*err == false))
        && (fifo->readPtr != fifo->writePtr)) {
        bool_t finished = false;
        uint8_t step = 0;
        size_t ptr = fifo->readPtr;   /* current pointer (integer, 0 based) */
        uint8_t* c = &fifo->buf[ptr]; /* current character */
        do {
            switch (step) {
                case 0: /* skip leading empty characters, stop on delimiter */
                    if (isgraph((int)*c) != 0) {
                        if (*c == DELIM_COMMENT) {
                            delimCommentFound = true;
                        } else {
                            buf[tokenSize] = (char)*c;
                            tokenSize++;
                            step++;
                        }
                    } else if (*c == DELIM_COMMAND) {
                        delimCommandFound = true;
                    } else { /* MISRA C 2004 14.10 */
                    }
                    break;
                case 1: /* search for end of the token */
                    if (isgraph((int)*c) != 0) {
                        if (*c == DELIM_COMMENT) {
                            delimCommentFound = true;
                        } else if (tokenSize < count) {
                            buf[tokenSize] = (char)*c;
                            tokenSize++;
                        } else { /* MISRA C 2004 14.10 */
                        }
                    } else {
                        if (*c == DELIM_COMMAND) {
                            delimCommandFound = true;
                        }
                        step++;
                    }
                    break;
                case 2: /* skip trailing empty characters */
                    if (isgraph((int)*c) != 0) {
                        if (*c == DELIM_COMMENT) {
                            delimCommentFound = true;
                        } else {
                            fifo->readPtr = ptr;
                            finished = true;
                        }
                    } else if (*c == DELIM_COMMAND) {
                        delimCommandFound = true;
                    } else { /* MISRA C 2004 14.10 */
                    }
                    break;
                default:
                    /* MISRA C 2004 15.3 */
                    break;
            }
            if (delimCommentFound == true) {
                /* Comment delimiter found, clear all till end of the line. */
                fifo->readPtr = ptr;
                delimCommandFound = CO_fifo_CommSearch(fifo, true);
                finished = true;
            } else if (delimCommandFound) {
                /* command delimiter found, set readPtr behind it. */
                if (++ptr == fifo->bufSize) {
                    ptr = 0;
                }
                fifo->readPtr = ptr;
                finished = true;
            } else if (!finished) {
                /* find next character in the circular buffer */
                if (++ptr == fifo->bufSize) {
                    ptr = 0;
                    c = &fifo->buf[ptr];
                } else {
                    c++;
                }
                /* end, if buffer is now empty */
                if (ptr == fifo->writePtr) {
                    if (step == 2U) {
                        fifo->readPtr = ptr;
                    } else {
                        tokenSize = 0;
                    }
                    finished = true;
                }
            } else { /* MISRA C 2004 14.10 */
            }
        } while (!finished);
    }

    /* set 'err' return value */
    if ((err != NULL) && (*err == false)) {
        if ((tokenSize == count)
            || ((closed != NULL)
                && (((*closed == 1U) && (!delimCommandFound || (tokenSize == 0U)))
                    || ((*closed == 0U) && (delimCommandFound || (tokenSize == 0U)))))) {
            *err = true;
        }
    }
    /* set 'closed' return value */
    if (closed != NULL) {
        *closed = delimCommandFound ? 1U : 0U;
    }

    /* token was larger then size of the buffer, all was cleaned, return empty */
    if (tokenSize == count) {
        tokenSize = 0;
    }
    /* write string terminator character */
    if ((buf != NULL) && (count > tokenSize)) {
        buf[tokenSize] = '\0';
    }

    return tokenSize;
}
#endif /* (CO_CONFIG_FIFO) & CO_CONFIG_FIFO_ASCII_COMMANDS */

#if ((CO_CONFIG_FIFO)&CO_CONFIG_FIFO_ASCII_DATATYPES) != 0
/* Tables for mime-base64 encoding, as specified in RFC 2045, (without CR-LF, but one long string).
 * Base64 is used for encoding binary data into easy transferable printable characters. In general,
 * each three bytes of binary data are translated into four characters, where characters are
 * selected from 64 characters long table. See https://en.wikipedia.org/wiki/Base64 */
static const char base64EncTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static const uint8_t base64DecTable[] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 103, 101, 255, 255, 102, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 103, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 62,
    255, 255, 255, 63,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  255, 255, 255, 100, 255, 255, 255, 0,
    1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,
    23,  24,  25,  255, 255, 255, 255, 255, 255, 26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,
    39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  255, 255, 255, 255, 255};

size_t
CO_fifo_readU82a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint8_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 6U)) {
        (void)CO_fifo_read(fifo, &n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRIu8, n);
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readU162a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint16_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 8U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRIu16, CO_SWAP_16(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readU322a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint32_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 12U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRIu32, CO_SWAP_32(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readU642a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint64_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 20U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRIu64, CO_SWAP_64(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readX82a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint8_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 6U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "0x%02" PRIX8, (uint32_t)n);
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readX162a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint16_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 8U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "0x%04" PRIX16, (uint32_t)CO_SWAP_16(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readX322a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint32_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 12U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "0x%08" PRIX32, CO_SWAP_32(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readX642a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    uint64_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 20U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "0x%016" PRIX64, CO_SWAP_64(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readI82a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    int8_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 6U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRId8, n);
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readI162a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    int16_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 8U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRId16, CO_SWAP_16(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readI322a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    int32_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 13U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRId32, CO_SWAP_32(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readI642a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    int64_t n = 0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 23U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%" PRId64, CO_SWAP_64(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readR322a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    float32_t n = (float32_t)0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 20U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%g", (float32_t)CO_SWAP_32(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readR642a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    float64_t n = (float64_t)0;

    if (fifo == NULL) {
        return 0;
    }

    if ((CO_fifo_getOccupied(fifo) == sizeof(n)) && (count >= 30U)) {
        (void)CO_fifo_read(fifo, (uint8_t*)&n, sizeof(n), NULL);
        return (size_t)sprintf(buf, "%g", (float64_t)CO_SWAP_64(n));
    } else {
        return CO_fifo_readHex2a(fifo, buf, count, end);
    }
}

size_t
CO_fifo_readHex2a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    (void)end; /* unused */

    size_t len = 0;

    if ((fifo != NULL) && (count > 3U)) {
        /* Very first write is without leading space */
        if (!fifo->started) {
            uint8_t c;
            if (CO_fifo_getc(fifo, &c)) {
                len = (size_t)sprintf(&buf[0], "%02" PRIX8, (uint32_t)c);
                fifo->started = true;
            }
        }

        while ((len + 3U) < count) {
            uint8_t c;
            if (!CO_fifo_getc(fifo, &c)) {
                break;
            }
            len += (size_t)sprintf(&buf[len], " %02" PRIX8, (uint32_t)c);
        }
    }

    return len;
}

size_t
CO_fifo_readVs2a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    size_t len = 0;

    if ((fifo != NULL) && (count > 3U)) {
        /* Start with '"' */
        if (!fifo->started) {
            buf[len] = '"';
            len++;
            fifo->started = true;
        }

        while ((len + 2U) < count) {
            uint8_t c;
            if (!CO_fifo_getc(fifo, &c)) {
                if (end) {
                    buf[len] = '"';
                    len++;
                }
                break;
            } else if ((c != 0U) && (c != (uint8_t)'\r')) {
                /* skip null and CR inside string */
                buf[len] = (char)c;
                len++;
                if (c == DELIM_DQUOTE) {
                    buf[len] = '"';
                    len++;
                }
            } else { /* MISRA C 2004 14.10 */
            }
        }
    }

    return len;
}

size_t
CO_fifo_readB642a(CO_fifo_t* fifo, char* buf, size_t count, bool_t end) {
    /* mime-base64 encoding, see description above base64EncTable */

    size_t len = 0;

    if ((fifo != NULL) && (count >= 4U)) {
        uint8_t step;
        uint16_t word;

        if (!fifo->started) {
            fifo->started = true;
            step = 0;
            word = 0;
        } else {
            /* get memorized variables from previous function calls */
            step = (uint8_t)(fifo->aux >> 16);
            word = (uint16_t)fifo->aux;
        }

        while ((len + 3U) <= count) {
            uint8_t c;

            if (!CO_fifo_getc(fifo, &c)) {
                /* buffer is empty, is also SDO communication finished? */
                if (end) {
                    /* add padding if necessary */
                    switch (step) {
                        case 1:
                            buf[len] = base64EncTable[(word >> 4) & 0x3FU];
                            len++;
                            buf[len] = '=';
                            len++;
                            buf[len] = '=';
                            len++;
                            break;
                        case 2:
                            buf[len] = base64EncTable[(word >> 6) & 0x3FU];
                            len++;
                            buf[len] = '=';
                            len++;
                            break;
                        default:
                            /* MISRA C 2004 15.3 */
                            break;
                    }
                }
                break;
            }

            word |= c;

            switch (step++) {
                case 0:
                    buf[len] = base64EncTable[(word >> 2) & 0x3FU];
                    len++;
                    break;
                case 1:
                    buf[len] = base64EncTable[(word >> 4) & 0x3FU];
                    len++;
                    break;
                default:
                    buf[len] = base64EncTable[(word >> 6) & 0x3FU];
                    len++;
                    buf[len] = base64EncTable[word & 0x3FU];
                    len++;
                    step = 0;
                    break;
            }
            word <<= 8;
        }

        /* memorize variables for next iteration */
        fifo->aux = ((uint32_t)step << 16) | word;
    }

    return len;
}

size_t
CO_fifo_cpyTok2U8(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[15];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        uint32_t u32 = strtoul(buf, &sRet, 0);
        if ((sRet != strchr(buf, (int32_t)('\0'))) || (u32 > (uint32_t)UINT8_MAX)) {
            st |= CO_fifo_st_errVal;
        } else {
            uint8_t num = (uint8_t)u32;
            nWr = CO_fifo_write(dest, &num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2U16(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[15];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        uint32_t u32 = strtoul(buf, &sRet, 0);
        if ((sRet != strchr(buf, (int32_t)('\0'))) || (u32 > (uint32_t)UINT16_MAX)) {
            st |= CO_fifo_st_errVal;
        } else {
            uint16_t num = CO_SWAP_16((uint16_t)u32);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2U32(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[15];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        uint32_t u32 = strtoul(buf, &sRet, 0);
        if (sRet != strchr(buf, (int32_t)('\0'))) {
            st |= CO_fifo_st_errVal;
        } else {
            uint32_t num = CO_SWAP_32(u32);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2U64(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[25];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        uint64_t u64 = strtoull(buf, &sRet, 0);
        if (sRet != strchr(buf, (int32_t)('\0'))) {
            st |= CO_fifo_st_errVal;
        } else {
            uint64_t num = CO_SWAP_64(u64);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = (uint8_t)st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2I8(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[15];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        int32_t i32 = strtol(buf, &sRet, 0);
        if ((sRet != strchr(buf, (int32_t)('\0'))) || (i32 < INT8_MIN) || (i32 > INT8_MAX)) {
            st |= CO_fifo_st_errVal;
        } else {
            int8_t num = (int8_t)i32;
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2I16(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[15];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        int32_t i32 = strtol(buf, &sRet, 0);
        if ((sRet != strchr(buf, (int32_t)('\0'))) || (i32 < INT16_MIN) || (i32 > INT16_MAX)) {
            st |= CO_fifo_st_errVal;
        } else {
            int16_t num = CO_SWAP_16((int16_t)i32);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2I32(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[15];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        int32_t i32 = strtol(buf, &sRet, 0);
        if (sRet != strchr(buf, (int32_t)('\0'))) {
            st |= CO_fifo_st_errVal;
        } else {
            int32_t num = CO_SWAP_32(i32);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2I64(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[25];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        int64_t i64 = strtoll(buf, &sRet, 0);
        if (sRet != strchr(buf, (int32_t)('\0'))) {
            st |= CO_fifo_st_errVal;
        } else {
            int64_t num = CO_SWAP_64(i64);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = (uint8_t)st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2R32(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[30];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        float32_t f32 = strtof(buf, &sRet);
        if (sRet != strchr(buf, (int32_t)('\0'))) {
            st |= CO_fifo_st_errVal;
        } else {
            float32_t num = CO_SWAP_32(f32);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2R64(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    char buf[40];
    uint8_t closed = 0xFFU;
    bool_t err = false;
    size_t nWr = 0;
    size_t nRd = CO_fifo_readToken(src, buf, sizeof(buf), &closed, &err);
    uint8_t st = closed;
    if ((nRd == 0U) || err) {
        st |= CO_fifo_st_errTok;
    } else {
        char* sRet;
        float64_t f64 = strtof(buf, &sRet);
        if (sRet != strchr(buf, (int32_t)('\0'))) {
            st |= CO_fifo_st_errVal;
        } else {
            float64_t num = CO_SWAP_64(f64);
            nWr = CO_fifo_write(dest, (uint8_t*)&num, sizeof(num), NULL);
            if (nWr != sizeof(num)) {
                st |= CO_fifo_st_errBuf;
            }
        }
    }
    if (status != NULL) {
        *status = st;
    }
    return nWr;
}

size_t
CO_fifo_cpyTok2Hex(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    size_t destSpace, destSpaceStart;
    bool_t finished = false;
    uint8_t step;
    uint8_t firstChar;
    uint8_t st = 0;

    if ((dest == NULL) || (src == NULL)) {
        return 0;
    }

    /* get free space of the dest fifo */
    destSpaceStart = CO_fifo_getSpace(dest);
    destSpace = destSpaceStart;

    /* is this the first write into dest? */
    if (!dest->started) {
        bool_t insideComment = false;
        if (CO_fifo_trimSpaces(src, &insideComment) || insideComment) {
            /* command delimiter found without string, this is an error */
            st |= CO_fifo_st_errTok;
        }
        dest->started = true;
        step = 0;
        firstChar = 0;
    } else {
        /* get memorized variables from previous function calls */
        step = (uint8_t)(dest->aux >> 8);
        firstChar = (uint8_t)(dest->aux & 0xFFU);
    }

    /* repeat until destination space available and no error and not finished
     * and source characters available */
    while ((destSpace > 0U) && ((st & CO_fifo_st_errMask) == 0U) && !finished) {
        uint8_t c;
        if (!CO_fifo_getc(src, &c)) {
            break;
        }

        if (step == 6U) {
            /* command is inside comment, waiting for command delimiter */
            bool_t insideComment = true;
            if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                st |= CO_fifo_st_closed;
                finished = true;
            }
            continue;
        }

        if ((int32_t)(isxdigit((int32_t)c)) != 0) {
            /* first or second hex digit */
            if (step == 0U) {
                firstChar = c;
                step = 1;
            } else {
                /* write the byte */
                uint8_t s[3];
                int32_t num;
                s[0] = firstChar;
                s[1] = c;
                s[2] = 0;
                num = strtol((char*)&s[0], NULL, 16);
                (void)CO_fifo_putc(dest, (uint8_t)num);
                destSpace--;
                step = 0;
            }
        } else if ((int32_t)(isgraph((int32_t)c)) != 0) {
            /* printable character, not hex digit */
            if (c == DELIM_COMMENT) { /* comment start */
                step = 6;
            } else { /* syntax error */
                st |= CO_fifo_st_errTok;
            }
        } else {
            /* this is space or delimiter */
            if (step == 1U) {
                /* write the byte */
                uint8_t s[2];
                int32_t num;
                s[0] = firstChar;
                s[1] = 0;
                num = strtol((char*)&s[0], NULL, 16);
                (void)CO_fifo_putc(dest, (uint8_t)num);
                destSpace--;
                step = 0;
            }
            bool_t insideComment = false;
            if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                /* newline found, finish */
                st |= CO_fifo_st_closed;
                finished = true;
            } else if (insideComment) {
                step = 6;
            } else { /* MISRA C 2004 14.10 */
            }
        }
    } /* while ... */

    if (!finished) {
        st |= CO_fifo_st_partial;
        /* memorize variables for next iteration */
        dest->aux = ((uint32_t)step << 8) | firstChar;
    }

    if (status != NULL) {
        *status = st;
    }

    return destSpaceStart - destSpace;
}

size_t
CO_fifo_cpyTok2Vs(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    size_t destSpace, destSpaceStart;
    bool_t finished = false;
    uint8_t step;
    uint8_t st = 0;

    if ((dest == NULL) || (src == NULL)) {
        return 0;
    }

    /* get free space of the dest fifo */
    destSpaceStart = CO_fifo_getSpace(dest);
    destSpace = destSpaceStart;

    /* is this the first write into dest? */
    if (!dest->started) {
        bool_t insideComment = false;
        if (CO_fifo_trimSpaces(src, &insideComment) || insideComment) {
            /* command delimiter found without string, this is an error */
            st |= CO_fifo_st_errTok;
        }
        dest->started = true;
        step = 0;
    } else {
        /* get memorized variables from previous function calls */
        step = (uint8_t)dest->aux;
    }

    /* repeat until destination space available and no error and not finished and source characters available */
    while ((destSpace > 0U) && ((st & CO_fifo_st_errMask) == 0U) && !finished) {
        uint8_t c;
        if (!CO_fifo_getc(src, &c)) {
            break;
        }

        switch (step) {
            case 0: /* beginning of the string, first write into dest */
                if (c == DELIM_DQUOTE) {
                    /* Indicated beginning of the string, skip this character. */
                    step = 1;
                } else {
                    /* this must be a single word string without '"' */
                    /* copy the character */
                    (void)CO_fifo_putc(dest, c);
                    destSpace--;
                    step = 2;
                }
                break;

            case 1: /* inside string, quoted string */
            case 2: /* inside string, single word, no quotes */
                if (c == DELIM_DQUOTE) {
                    /* double quote found, this may be end of the string or escaped
                     * double quote (with two double quotes) */
                    step += 2U;
                } else if ((isgraph((int)c) == 0) && (step == 2U)) {
                    /* end of single word string */
                    bool_t insideComment = false;
                    if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                        st |= CO_fifo_st_closed;
                        finished = true;
                    } else {
                        step = insideComment ? 6U : 5U;
                    }
                } else if (c == DELIM_COMMAND) {
                    /* no closing quote, error */
                    st |= CO_fifo_st_errTok;
                } else {
                    /* copy the character */
                    (void)CO_fifo_putc(dest, c);
                    destSpace--;
                }
                break;

            case 3: /* previous was double quote, parsing quoted string */
            case 4: /* previous was double quote, parsing no quoted word */
                if (c == DELIM_DQUOTE) {
                    /* escaped double quote, copy the character and continue */
                    (void)CO_fifo_putc(dest, c);
                    destSpace--;
                    step -= 2U;
                } else {
                    /* previous character was closing double quote */
                    if (step == 4U) {
                        /* no opening double quote, syntax error */
                        st |= CO_fifo_st_errTok;
                    } else {
                        if (isgraph((int)c) == 0) {
                            /* end of quoted string */
                            bool_t insideComment = false;
                            if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                                st |= CO_fifo_st_closed;
                                finished = true;
                            } else {
                                step = insideComment ? 6U : 5U;
                            }
                        } else {
                            /* space must follow closing double quote, error */
                            st |= CO_fifo_st_errTok;
                        }
                    }
                }
                break;

            case 5: { /* String token is finished, waiting for command delimiter */
                bool_t insideComment = false;
                if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                    st |= CO_fifo_st_closed;
                    finished = true;
                } else if (insideComment) {
                    step = 6;
                } else if (isgraph((int)c) != 0) {
                    if (c == DELIM_COMMENT) { /* comment start */
                        step = 6;
                    } else { /* syntax error */
                        st |= CO_fifo_st_errTok;
                    }
                } else { /* MISRA C 2004 14.10 */
                }
                break;
            }
            case 6: { /* String token is finished, waiting for command delimiter */
                bool_t insideComment = true;
                if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                    st |= CO_fifo_st_closed;
                    finished = true;
                }
                break;
            }
            default: /* internal error */ st |= CO_fifo_st_errInt; break;
        }
    }

    if (!finished) {
        st |= CO_fifo_st_partial;
        /* memorize variables for next iteration */
        dest->aux = step;
    }

    if (status != NULL) {
        *status = st;
    }

    return destSpaceStart - destSpace;
}

size_t
CO_fifo_cpyTok2B64(CO_fifo_t* dest, CO_fifo_t* src, uint8_t* status) {
    /* mime-base64 decoding, see description above base64EncTable */

    size_t destSpace, destSpaceStart;
    bool_t finished = false;
    uint8_t step;
    uint32_t dword;
    uint8_t st = 0;

    if ((dest == NULL) || (src == NULL)) {
        return 0;
    }

    /* get free space of the dest fifo */
    destSpaceStart = CO_fifo_getSpace(dest);
    destSpace = destSpaceStart;

    /* is this the first write into dest? */
    if (!dest->started) {
        bool_t insideComment = false;
        if (CO_fifo_trimSpaces(src, &insideComment) || insideComment) {
            /* command delimiter found without string, this is an error */
            st |= CO_fifo_st_errTok;
        }
        dest->started = true;
        step = 0;
        dword = 0;
    } else {
        /* get memorized variables from previous function calls */
        step = (uint8_t)(dest->aux >> 24);
        dword = dest->aux & 0xFFFFFFU;
    }

    /* repeat until destination space available and no error and not finished and source characters available */
    while ((destSpace >= 3U) && ((st & CO_fifo_st_errMask) == 0U) && !finished) {
        uint8_t c;
        if (!CO_fifo_getc(src, &c)) {
            break;
        }

        if (step >= 5U) {
            /* String token is finished, waiting for command delimiter */
            bool_t insideComment = step > 5U;
            if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                st |= CO_fifo_st_closed;
                finished = true;
            } else if (insideComment) {
                step = 6;
            } else if ((isgraph((int)c) != 0) && (c != (uint8_t)'=')) {
                if (c == DELIM_COMMENT) { /* comment start */
                    step = 6;
                } else { /* syntax error */
                    st |= CO_fifo_st_errTok;
                }
            } else { /* MISRA C 2004 14.10 */
            }
            continue;
        }

        uint8_t code = base64DecTable[c & 0x7FU];

        if (((c & 0x80U) != 0U) || ((code & 0x80U) != 0U)) {
            st |= CO_fifo_st_errTok;
        } else if (code >= 64U /* '=' (pad) or DELIM_COMMAND or space */) {
            /* base64 string finished, write remaining bytes */
            switch (step) {
                case 2:
                    (void)CO_fifo_putc(dest, (uint8_t)(dword >> 4));
                    destSpace--;
                    break;
                case 3:
                    (void)CO_fifo_putc(dest, (uint8_t)(dword >> 10));
                    (void)CO_fifo_putc(dest, (uint8_t)(dword >> 2));
                    destSpace -= 2U;
                    break;
                default:
                    /* MISRA C 2004 15.3 */
                    break;
            }

            bool_t insideComment = false;
            if (CO_fifo_trimSpaces(src, &insideComment) || (c == DELIM_COMMAND)) {
                st |= CO_fifo_st_closed;
                finished = true;
            } else {
                step = insideComment ? 6U : 5U;
            }
        } else {
            dword = (dword << 6) | code;
            if (step++ == 3U) {
                (void)CO_fifo_putc(dest, (uint8_t)((dword >> 16) & 0xFFU));
                (void)CO_fifo_putc(dest, (uint8_t)((dword >> 8) & 0xFFU));
                (void)CO_fifo_putc(dest, (uint8_t)(dword & 0xFFU));
                destSpace -= 3U;
                dword = 0;
                step = 0;
            }
        }
    } /* while ... */

    if (!finished) {
        st |= CO_fifo_st_partial;
        /* memorize variables for next iteration */
        dest->aux = ((uint32_t)step << 24) | (dword & 0xFFFFFFU);
    }

    if (status != NULL) {
        *status = st;
    }

    return destSpaceStart - destSpace;
}

#endif /* (CO_CONFIG_FIFO) & CO_CONFIG_FIFO_ASCII_DATATYPES */

#endif /* (CO_CONFIG_FIFO) & CO_CONFIG_FIFO_ENABLE */
