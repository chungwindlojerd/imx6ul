#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <linux/ioctl.h>

int main(int argc, char *argv[])
{
    int fd = 0;
    int ret = 0;
    char *filename = NULL;
    unsigned char data;
    
    if (argc < 2) 
    {
        printf("error usage.\r\n");
        return -1;
    }

    filename = argv[1];
    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        printf("can't open file %s\r\n", filename);
        return -1;
    }

    while (1)
    {
        ret = read(fd, &data, sizeof(data));
        if (ret < 0)
        {
            
        }
        else
        {
            if (data)
            {
                printf("key value = %#X\r\n", data);
            }
        }
    }

    close(fd);
    return 0;
}