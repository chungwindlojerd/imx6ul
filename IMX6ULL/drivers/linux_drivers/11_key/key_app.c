#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#define KEY0VALUE           0xF0
#define INVAKEY             0x00


int main(int argc, char *argv[])
{
    int fd = 0;
    int ret = 0;
    char *filename;
    unsigned char keyvalue;

    if (argc != 2)
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

    while (1)
    {
        read(fd, &keyvalue, sizeof(keyvalue));
        if (keyvalue == KEY0VALUE)
        {
            printf("KEY0 press, value = %#x\r\n", keyvalue);
        }
    }

    ret = close(fd);
    if (ret < 0)
    {
        printf("file %s close failed.\r\n", filename);
        return -1;
    }

    return 0;
}

