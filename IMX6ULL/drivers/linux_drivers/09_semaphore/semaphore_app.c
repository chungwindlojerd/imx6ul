#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#define LED_ON      1
#define LED_OFF     0

int main(int argc, char *argv[])
{
    int fd = 0;
    int retvalue = 0;
    char *filename = NULL;
    unsigned char cnt = 0;
    unsigned char databuf[1];

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

    databuf[0] = atoi(argv[2]);

    retvalue = write(fd, databuf, sizeof(databuf));
    if (retvalue < 0)
    {
        printf("led control failed.\r\n");
        close(fd);
        return -1;
    }

    while (1)
    {
        sleep(5);
        cnt ++;
        printf("app running times: %d\r\n", cnt);
        if (cnt > 5) break;
    }

    printf("app running finishe.\r\n");
    retvalue = close(fd);

    if (retvalue < 0)
    {
        printf("file %s close failed.\r\n", filename);
        return -1;
    }
    return 0;
}