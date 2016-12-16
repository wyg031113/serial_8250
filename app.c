/*************************************************************************
    > File Name: app.c
    > Author: wyg
    > Mail: wyg_0802@126.com 
    > Created Time: 2016年12月10日 星期六 10时13分02秒
 ************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

int main(int argc, char *argv[])
{
	if(argc != 2){
		printf("Usage:./app /dev/serial0\n");
		return -1;
	}
	int fd = open(argv[1], O_RDWR);
	if(fd < 0){
		printf("open memdev0 failed:%s.\n", strerror(errno));
		return -1;
	}
	sleep(30);
	printf("open memdev0 successfully!\n");
	close(fd);
	return 0;
}
