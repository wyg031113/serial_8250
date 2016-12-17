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
#include <time.h>
#include <stdlib.h>
int main(int argc, char *argv[])
{
	if(argc != 3){
		printf("Usage:./send file /dev/serial0\n");
		return -1;
	}
	int fd = open(argv[1], O_RDONLY);
	int wd = open(argv[2], O_WRONLY);
	static char buf[4096];
	int len = 0;
	long long total = 0;
	if(fd < 0 || wd < 0){
		printf("open  failed:%s.\n", strerror(errno));
		return -1;
	}
	time_t start = time(NULL);
	while(1){
		len = read(fd, buf, 16);
		if(len <= 0)
			break;
		int ret = 0;
		int cnt = 0;
		while(len > 0){
			ret = write(wd, buf+cnt, len);
			usleep(10000);
			if(ret < 0)
				break;
			len -= ret;
			cnt += ret;
		}
		//if(len != ret)
		//	break;
		total += len;
		time_t pst = time(NULL) - start;
		if(pst == 0)
			pst ++;
		printf("read %d bytes\nwrite %d bytes\ntotal:%lld\n speed:%lld bytes/s\n", cnt, ret, total, total/(long long)pst);
	}
	sleep(1);
	printf("open memdev0 successfully!\n");
	close(fd);
	return 0;
}
