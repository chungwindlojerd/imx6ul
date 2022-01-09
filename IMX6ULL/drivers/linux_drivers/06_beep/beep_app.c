/**
 * @file beepApp.c
 * @author Jack (jackhuang021@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#define BEEP_OFF        0
#define BEEP_ON         1

int main(int argc, char *argv[])
{
    int fd = 0;
    int retvalue = 0;
    char *filename;
    unsigned char databuff[1];

    if (argc != 3)
    {
        printf("error usage.\r\n");
        return -1;
    }

    filename = argv[1];

    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        printf("file %s open failed.\r\n", filename);
        return -1;
    }

    databuff[0] = atoi(argv[2]);

    retvalue = write(fd, databuff, sizeof(databuff));
    if (retvalue < 0)
    {
        printf("beep control failed.\r\n");
        close(fd);
        return -1;
    }

    retvalue = close(fd);
    if (retvalue < 0)
    {
        printf("file %s close failed.\r\n", filename);
        return -1;
    }
    return 0;
}