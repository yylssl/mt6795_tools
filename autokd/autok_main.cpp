#include <cstdio>
#include <cstdlib>
#include "autok.h"
#include <cstring>
#include <unistd.h>
unsigned int *g_autok_vcore;
int g_vcore_no;// = (sizeof(g_autok_vcore)/sizeof(unsigned int));

int main(int argc, char *argv[])
{
    int result = 0;
    int is_ss_corner = 0;
    unsigned int *vol_list;
    int vol_count = 0;
    int i;

    LOGD("AUTOKD Start\n");    
          
    g_autok_vcore = (unsigned int*)malloc(sizeof(unsigned int)*3);
    g_autok_vcore[0] = 1187500;
    g_autok_vcore[1] = 1237500;
    g_autok_vcore[2] = 1281250;
    
    is_ss_corner = get_ss_corner();
    if(is_ss_corner)
        g_autok_vcore[0] = 1150000;
    g_vcore_no = 3;
        
    autok_flow();
    close_nvram();
	return result;
}
