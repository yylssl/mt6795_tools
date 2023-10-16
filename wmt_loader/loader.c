#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <time.h>

#define WCN_COMBO_LOADER_DEV "/dev/wmtdetect"
#define MTK_WCN_DRIVER_READY "/tmp/mtk_wcn_driver_ready"

#define WMT_IOC_MAGIC                        'w' // 119
#define COMBO_IOCTL_GET_CHIP_ID       _IOR(WMT_IOC_MAGIC, 0, int)
#define COMBO_IOCTL_SET_CHIP_ID       _IOW(WMT_IOC_MAGIC, 1, int)
#define COMBO_IOCTL_EXT_CHIP_DETECT   _IOR(WMT_IOC_MAGIC, 2, int)
#define COMBO_IOCTL_GET_SOC_CHIP_ID   _IOR(WMT_IOC_MAGIC, 3, int)
#define COMBO_IOCTL_DO_MODULE_INIT    _IOR(WMT_IOC_MAGIC, 4, int)
#define COMBO_IOCTL_MODULE_CLEANUP    _IOR(WMT_IOC_MAGIC, 5, int)
#define COMBO_IOCTL_EXT_CHIP_PWR_ON   _IOR(WMT_IOC_MAGIC, 6, int)
#define COMBO_IOCTL_EXT_CHIP_PWR_OFF  _IOR(WMT_IOC_MAGIC, 7, int)
#define COMBO_IOCTL_DO_SDIO_AUDOK     _IOR(WMT_IOC_MAGIC, 8, int)

int do_kernel_module_init(int loaderFd, int chipId) {
    int ret = 0;
    if (loaderFd < 0) {
        printf("invalid loaderFd: %d\n", loaderFd);
        return -1;
    }

    ret = ioctl (loaderFd, COMBO_IOCTL_MODULE_CLEANUP, chipId);
    if (ret) {
        printf("do WMT-DETECT module cleanup failed: %d\n", ret);
        return -2;
    }

    ret = ioctl (loaderFd, COMBO_IOCTL_DO_MODULE_INIT, chipId);
    if (ret) {
        printf("do kernel module init failed: %d\n", ret);
        return -3;
    }

    printf("do kernel module init succeed: %d\n", ret);
    return 0;
}

int set_wcn_driver_ready()
{
    int fd = open(MTK_WCN_DRIVER_READY, O_CREAT|O_TRUNC|O_WRONLY);
    if (fd < 0) {
        printf("open mtk wcn driver ready file %s failed\n", MTK_WCN_DRIVER_READY);
        return -1;
    }

    char timestamp[32] = {0};
    snprintf(timestamp, sizeof(timestamp), "%ld", (int64_t)time(NULL));
    ssize_t len = write(fd, timestamp, strlen(timestamp));
    if (len != strlen(timestamp)) {
        printf("write mtk wcn driver ready file %s failed\n", MTK_WCN_DRIVER_READY);
    }

    close(fd);
    return 0;
}

int main(int argc, char *argv[]) {
    int ret = -1;

	/* open wcn combo loader dev */
	int loaderFd = -1;
    do {
        loaderFd = open(WCN_COMBO_LOADER_DEV, O_RDWR | O_NOCTTY);
        if(loaderFd < 0) {
            printf("Can't open device node(%s)\n", WCN_COMBO_LOADER_DEV);
            usleep(300000);
        } else {
            break;
        } 
    }while(1);

    /*trigger external combo chip detect and chip identification process*/
	int chipId = -1;
	int retryCounter = 1;
    do {
		int noextChip = -1;

        /*power on combo chip*/
        ret = ioctl(loaderFd,COMBO_IOCTL_EXT_CHIP_PWR_ON);
        if (0 != ret) {
            printf("external combo chip power on failed\n");
            noextChip = 1;
        } else {
            /*detect is there is an external combo chip*/
            noextChip = ioctl(loaderFd,COMBO_IOCTL_EXT_CHIP_DETECT,NULL);
        }

        if(noextChip) { // use soc itself
            printf("no external combo chip detected, get current soc chipid\n");
            chipId = ioctl(loaderFd, COMBO_IOCTL_GET_SOC_CHIP_ID, NULL);
            printf("soc chipid (0x%x) detected\n", chipId);
        } else {
            printf("external combo chip detected\n");
            chipId = ioctl(loaderFd, COMBO_IOCTL_GET_CHIP_ID, NULL);
            printf("chipid (0x%x) detected\n", chipId);
        }

        if(0 == noextChip) {
            ret = ioctl(loaderFd,COMBO_IOCTL_DO_SDIO_AUDOK,chipId);
            if (0 != ret) {
                printf("do SDIO3.0 autok failed\n");
            } else {
                printf("do SDIO3.0 autok succeed\n");
            }
        }
        
        ret = ioctl(loaderFd,COMBO_IOCTL_EXT_CHIP_PWR_OFF);
        if (0 != ret) {
            printf("external combo chip power off failed\n");
        } else {
            printf("external combo chip power off succeed\n");
        }
        if ((0 == noextChip) && (-1 == chipId)) {
            /*extenral chip detected, but no valid chipId detected, retry*/
            retryCounter--;
            printf("chipId detect failed, retrying, left retryCounter:%d\n", retryCounter);
            usleep(500000);
        } else {
            break;
        }
    }while (0 < retryCounter);

    /*set chipid to kernel*/
    ioctl(loaderFd,COMBO_IOCTL_SET_CHIP_ID,chipId);

    if((0x0321 == chipId) || (0x0335 == chipId) || (0x0337 == chipId)) {
        chipId = 0x6735;
    }
    if (0x0326 == chipId) {
        chipId = 0x6755;
    }
    if (0x0279 == chipId) {
        chipId = 0x6797;
    }
    do_kernel_module_init(loaderFd, chipId);
    
	if(loaderFd >= 0) {
        close(loaderFd);
        loaderFd = -1;
    }

    if((chown("/proc/driver/wmt_dbg",0,1000) == -1) || (chown("/proc/driver/wmt_aee",0,1000) == -1)) {
        printf("chown wmt_dbg or wmt_aee fail:%s\n",strerror(errno));
    }

    if(chown("/proc/wmt_tm/wmt_tm",0,1000) == -1) {
        printf("chown wmt_tm fail:%s\n",strerror(errno));
    }

    set_wcn_driver_ready();

    printf("wcn driver ready succeed\n");
    return ret;
}