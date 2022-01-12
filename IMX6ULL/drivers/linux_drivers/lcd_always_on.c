#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>

int main(int argc, char *argv[])
{
    int fd;
    fd = open("/dev/tty1", O_RDWR);
    write(fd, "\033[9;0]", 8);
    close(fd);
    return 0;
}