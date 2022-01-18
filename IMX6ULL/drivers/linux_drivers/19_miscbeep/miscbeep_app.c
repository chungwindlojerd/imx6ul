#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#define BEEP_OFF    0
#define BEEP_ON     1


int main(int argc, char *argv[])
{
    int fd = 0;
    int ret_value = 0;
    char *filename;
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
    ret_value = write(fd, databuf, sizeof(databuf));
    if (ret_value < 0)
    {
        printf("beep control failed.\r\n");
        return -1;
    }

    ret_value = close(fd);
    if (ret_value < 0)
    {
        printf("file %s close failed.\r\n", filename);
        return -1;
    }

    return 0;
}