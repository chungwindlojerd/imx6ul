#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <linux/ioctl.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>

int main(int argc, char *argv[])
{
    int fd = 0;
    int ret = 0;
    char *filename = NULL;
    struct pollfd fds;
    fd_set readfds;
    struct timeval timeout;
    unsigned char data;
    
    if (argc != 2) 
    {
        printf("error usage.\r\n");
        return -1;
    }

    filename = argv[1];
    fd = open(filename, O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        printf("can't open file %s\r\n", filename);
        return -1;
    }

    while (1)
    {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 500000;
        ret = select(fd + 1, &readfds, NULL, NULL, &timeout);
        switch (ret)
        {
            case 0: break;
            case -1: break;
            default: 
                if (FD_ISSET(fd, &readfds))
                {
                    ret = read(fd, &data, sizeof(data));
                    if (ret < 0)
                    {

                    }
                    else
                    {
                        if (data) printf("key value = %d\r\n", data);
                    }
                }
                break;
        }
    }

    close(fd);
    return 0;
}