#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <linux/ioctl.h>
#include <string.h>
#include <linux/ioctl.h>

#define CLOSE_CMD       (_IO(0xFE, 0x01))
#define OPEN_CMD        (_IO(0xFE, 0x02))
#define SETPERIOD_CMD   (_IO(0xFE, 0x03))

int main(int argc, char *argv[])
{
    int fd = 0;
    int ret = 0;
    const char *filename = NULL;
    unsigned int cmd;
    unsigned int arg;
    unsigned char str[100];

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
        printf("input cmd: ");
        ret = scanf("%d", cmd);

        if (cmd == 1) cmd = CLOSE_CMD;
        else if (cmd == 2) cmd = OPEN_CMD;
        else if (cmd == 3) 
        {
            cmd = SETPERIOD_CMD;
            printf("input timer period: ");
            ret = scanf("%d", &arg);
        }
        ioctl(fd, cmd, arg);
    }
    close(fd);
}