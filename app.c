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
int main(int argc, char *argv[])
{
	if(argc != 2){
		printf("Usage:./app /dev/serial0\n");
		return -1;
	}
	int fd = open(argv[1], O_RDWR);
	static char buf[4096];
	int len = 0;
	long long total = 0;
	if(fd < 0){
		printf("open  failed:%s.\n", strerror(errno));
		return -1;
	}
	time_t start = time(NULL);
	while(1){
		len = read(fd, buf, 4096);
		total += len;
		int ret = 0;
		int tlen = len;
		printf("read %d bytes\n", len);
		while(len > 0){
			ret = write(fd, buf+(tlen-len), len);
			if(ret < 0)
				break;
			len -= ret;
		}
		time_t pst = time(NULL) - start;
		printf("read %d bytes\nwrite %d bytes\ntotal:%lld\n speed:%lld bytes/s\n", len, ret, total, total/(long long)pst);
	}
	printf("open memdev0 successfully!\n");
	close(fd);
	return 0;
}
