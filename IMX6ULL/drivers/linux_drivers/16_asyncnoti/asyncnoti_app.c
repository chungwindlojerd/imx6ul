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
#include <linux/ioctl.h>
#include <signal.h>

static int fd = 0;

static void sigio_signal_func(int signum)
{
    int err = 0;
    unsigned int keyvalue = 0;
    err = read(fd, &keyvalue, sizeof(keyvalue));
    if (err < 0)
    {
        
    }
    else
    {
        printf("sigio signal, key value = %d\r\n", keyvalue);
    }
}


int main(int argc, char *argv[])
{
    int flags = 0;
    char *filename = NULL;
    
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

    signal(SIGIO, sigio_signal_func);
    fcntl(fd, F_SETOWN, getpid());
    flags = fcntl(fd, F_GETFD);
    fcntl(fd, F_SETFL, flags | FASYNC);

    while (1)
    {
        sleep(2);
    }

    close(fd);
    return 0;
}