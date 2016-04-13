#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <strings.h>
#include <string.h>
#include <time.h>


#define HANDEL_ERROR(x, y) if(x < 0) { printf("Error: %s\n", y); exit(1); }
#define ERROR(x) printf("Error: %s\n", x); exit(1);

char * portName = "/dev/ttyACM0";

void flushBuffer(int fd);
void writeCommand(int fd, const char * command);
void printBuffer(int fd);
int decode(int fd, uint8_t nBytes);
void decodeData(int fd);

int main() {

    int fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
    HANDEL_ERROR(fd, "Could not open port");
    flushBuffer(fd);
    writeCommand(fd, "RS");
    if(decode(fd, 2) != 0) {
        ERROR("Could not reset sensor");
    }
    writeCommand(fd, "HS1");
    if(decode(fd, 2) != 0) {
        ERROR("Could not set the sensor to high sensitivity");
    }
    writeCommand(fd, "BM");
    int result = decode(fd, 2);
    if(result != 0 && result != 2) {
        printf("Got error code %d\n", result);
        ERROR("Could not enable the laser");
    }
    flushBuffer(fd);

    while(1) {
        writeCommand(fd, "GD0000075000"); //Get scan from 0 to 750, 0 clusters
        if(decode(fd, 2) != 0) {
            ERROR("Could not read lidar frame");
        }
        decodeData(fd);
    }




    writeCommand(fd, "RS");
    close(fd);

    return 0;
}



void flushBuffer(int fd) {
    uint8_t c;
    while(read(fd, &c, 1) > 0) {}
}


void writeCommand(int fd, const char * command) {
    ssize_t n = write(fd, command, strlen(command));
    n += write(fd, "\n", 1);
    usleep(1000 * 100);
    char c;
    while(read(fd, &c, 1) > 0 && c != '\n') {}
}

void printBuffer(int fd) {
    char c;
    while(read(fd, &c, 1) > 0) {
        printf("%c", c);
    }
    printf("\n");
}

void decodeData(int fd) {
    char c;
    while(read(fd, &c, 1) > 0 && c != '\n') {}
    char data[750 * 3 + 10];
    int magnitudes[750];
    char * position = data;
    while(1) {
        ssize_t n = read(fd, position, 66);
        if(n < 66) {
            break;
        }
        position += n - 2;
    }
    data[750 * 3] = '\0';
    clock_t start = clock();
    for(int a = 0;a < 750;a++) {
        magnitudes[a]  = ((data[a*3 + 0] - 0x30) << (2*6));
        magnitudes[a] |= ((data[a*3 + 1] - 0x30) << (1*6));
        magnitudes[a] |= ((data[a*3 + 2] - 0x30) << (0*6));
        //printf("Magnitude: %d\n", magnitudes[a]);
    }
    printf("Loop took %10.9f distance at middle: %d\n", (double)(clock() - start) / CLOCKS_PER_SEC, magnitudes[750 / 2]);
}

int decode(int fd, uint8_t nBytes) {
    int result = 0;
    int sum = 0;
    char b[7]; //Max of 4 byte encoding + sum byte + two line feeds
    char * position = b;
    bzero(b, 5);
    int numberToRead = nBytes + 1 + 2;
    while(numberToRead > 0) {
        ssize_t n = read(fd, position, numberToRead);
        if(n < 0) {
            continue;
        }
        numberToRead -= n;
    }
    //printf("Received: %s\n", b);
    for(uint8_t a = 0;a < nBytes;a++) {
        result |= (b[a] - 0x30) << ((nBytes - a) * 6);
        sum += b[a];
    }
    sum = (sum & 0x3F) + 0x30;
    if(sum != b[nBytes]) {
        printf("Got: %s\n----\n", b);
        printf("Checksum failed\n");
    }
    return(result);
}