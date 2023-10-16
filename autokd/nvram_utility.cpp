
#include <cstdio>
#include "autok.h"
// #include "CFG_SDIO_File.h"
// #include "CFG_file_lid.h"
// #include "libnvram.h"
#include <list>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>

unsigned char *g_nv_buf;
#define EACH_FILE_SIZE (200)
#define MAX_COUNT (19)
struct autok_single_element
{
    unsigned char vol_count; //always 1
    unsigned char param_count;
    unsigned int voltage;       //1 voltage only
    U_AUTOK_INTERFACE_DATA *ai_data;
};

typedef struct
{
    unsigned char file_count;
    unsigned char id[19];
    unsigned int file_length[19];
    char data[3998];
} ap_nvram_sdio_config_struct;

int sdio_read_nvram(unsigned char *ucNvRamData)
{
    int rec_size = 0;
    int rec_num = 0;
    int sdio_cfgfile_fd = -1;
    ap_nvram_sdio_config_struct sdio_nvram;

    memset(&sdio_nvram, 0, sizeof(ap_nvram_sdio_config_struct));
    
    /* Try Nvram first */
    //sdio_nvram_fd = NVM_GetFileDesc(AP_CFG_RDCL_FILE_SDIO_LID, &rec_size, &rec_num, ISWRITE);
    FILE* inFile = fopen("/etc/nvram/SDIO", "rb");
    if(NULL != inFile) {
        {
            fseek (inFile , 0 , SEEK_END);
            int lSize = ftell (inFile);
            lSize = lSize > 0?lSize:0;
            rewind (inFile);

            rec_size = sizeof(ap_nvram_sdio_config_struct);
            rec_num  = lSize/rec_size;
        }
        
        if(rec_num != 1){
            LOGE("Unexpected record num %d", rec_num);
            fclose(inFile);
            return -1;
        }
        
        if(rec_size != sizeof(ap_nvram_sdio_config_struct)){
            LOGE("Unexpected record size %d ap_nvram_sdioradio_struct %d",
                    rec_size, sizeof(ap_nvram_sdio_config_struct));
            fclose(inFile);
            return -1;
        }
        
        if(fread(&sdio_nvram, 1, rec_num*rec_size, inFile) < 0){
            LOGE("Read NVRAM fails errno %d\n", errno);
            fclose(inFile);
            return -1;
        }
        
        fclose(inFile);
    }
    
    memcpy(ucNvRamData, &sdio_nvram, sizeof(ap_nvram_sdio_config_struct));
    
    return 0;
}


int read_from_nvram(int id, unsigned int voltage, unsigned char **file_data, int *length)
{
    int i;
    int file_length = 0;
    int file_count = 0;
    int result = -1;    
    ap_nvram_sdio_config_struct *nv_data = (ap_nvram_sdio_config_struct*)g_nv_buf;   
    
    
    file_count = nv_data->file_count;
    LOGI("file_count[%d]\n", file_count);
    for(i=0; i<file_count; i++){
        if(nv_data->id[i] == id){
            struct autok_single_element autok_ptr;
            // = (struct autok_single_element*)&nv_data->data[i*EACH_FILE_SIZE];
            //for(int j=0; j<10; j++)
            //LOGI("%02x", nv_data->data[i*EACH_FILE_SIZE+j]);
            char *ptr = &nv_data->data[i*EACH_FILE_SIZE];
            memcpy(&autok_ptr.vol_count, ptr, 1);
            memcpy(&autok_ptr.param_count, ptr+1, 1);
            memcpy(&autok_ptr.voltage, ptr+2, 4);
            //LOGI("voltage[%x][%d]", autok_ptr.voltage, voltage);
            if(autok_ptr.voltage == voltage){
                file_length = nv_data->file_length[i];
                //LOGI("filelength:%d\n", file_length);
                *file_data = (unsigned char*)malloc(sizeof(unsigned char)*file_length);
                memcpy(*file_data, &nv_data->data[i*EACH_FILE_SIZE], file_length);
                *length = file_length;
                //AUTOK_FREE(nv_buf);
                return 0;
            }
        }
    }
    //AUTOK_FREE(nv_buf);
    file_data = NULL;
    return 0;
}

int write_nvram(unsigned char* write_buf, int length, int id, int file_idx, int file_count)
{   
    FILE* outFile = fopen("/etc/nvram/SDIO", "wb");
    if(outFile != NULL){
        // Write to data
        fseek(outFile, 1+MAX_COUNT+(MAX_COUNT*sizeof(unsigned int))+(file_idx*EACH_FILE_SIZE), SEEK_SET);
        if(fwrite(write_buf, 1, length, outFile)<0){
            LOGE("Write NVRAM fails errno %d\n", errno);
            fclose(outFile);
            return -1;
        }        
        // Write to file_length
        fseek(outFile, 1+MAX_COUNT+(file_idx*sizeof(unsigned int)), SEEK_SET);
        if(fwrite(&length, 1, sizeof(unsigned char), outFile)<0){
            LOGE("Write NVRAM fails errno %d\n", errno);
            fclose(outFile);
            return -1;
        }
        // Write to host_id
        fseek(outFile, 1+file_idx, SEEK_SET);
        if(fwrite((unsigned char*)&id, 1, sizeof(unsigned char), outFile)<0){
            LOGE("Write NVRAM fails errno %d\n", errno);
            fclose(outFile);
            return -1;
        }
        // Write to file_count
        fseek(outFile, 0, SEEK_SET);
        if(fwrite((unsigned char*)&file_count, 1, sizeof(unsigned char), outFile)<0){
            LOGE("Write NVRAM fails errno %d\n", errno);
            fclose(outFile);
            return -1;
        }
        fclose(outFile);
    }
    return 0;
}

int write_dev_to_nvram(char *filename, int id)
{
    int file_count;
    int lSize;
    int i;  
    unsigned char *data_buf = NULL;
    
    get_node_data(filename, (char**)&data_buf, &lSize);
        
    struct autok_single_element *in_buf = (struct autok_single_element*)malloc(sizeof(struct autok_single_element));
    ap_nvram_sdio_config_struct *nv_data = (ap_nvram_sdio_config_struct*)g_nv_buf;   
    file_count = nv_data->file_count;
    memcpy(&in_buf->voltage, &data_buf[2], sizeof(unsigned int));
    // file update
    for(i=0; i<file_count; i++){
        if(nv_data->id[i] == id){
            struct autok_single_element *autok_ptr = (struct autok_single_element *)malloc(sizeof(struct autok_single_element));
            //; = (struct autok_single_element*)&nv_data->data[i*EACH_FILE_SIZE];
            memcpy(&autok_ptr->voltage, (&nv_data->data[i*EACH_FILE_SIZE])+2, sizeof(unsigned int));
            LOGI("nv_voltage[%d] buf_voltage[%d]\n", autok_ptr->voltage, in_buf->voltage);
            if(autok_ptr->voltage == in_buf->voltage){
              LOGI("file_update:%d", file_count);
                nv_data->file_length[i] = lSize;
                memcpy(&nv_data->data[i*EACH_FILE_SIZE], data_buf, lSize);
                write_nvram(data_buf, lSize, id, i, file_count);
                AUTOK_FREE(autok_ptr);
                break;
            }
            AUTOK_FREE(autok_ptr);
        }
    }
    AUTOK_FREE(in_buf);
    // new file(no found in table)
    if(i==file_count){
        LOGI("add new file with size:%d", lSize);
        nv_data->id[file_count] = id;
        nv_data->file_length[file_count] = lSize;
        memcpy(&nv_data->data[file_count*EACH_FILE_SIZE], data_buf, lSize);        
        write_nvram(data_buf, lSize, id, file_count, file_count+1);
        nv_data->file_count += 1;   // zero based   
    }    
    AUTOK_FREE(data_buf);
    return 0;
}

int write_file_to_nvram(char *filename, int id)
{
    unsigned int file_count;
    FILE * inFile;
    unsigned int i;
    struct autok_single_element *in_buf;
    ap_nvram_sdio_config_struct *nv_data;
    long lSize = 0; 
    unsigned char *data_buf = NULL;
    
    inFile = fopen (filename, "rb");
    if (inFile==NULL) {
        LOGE("File error"); 
        return -1;
    }
    fseek (inFile , 0 , SEEK_END);
    lSize = ftell (inFile);
    lSize = lSize > 0?lSize:0;
    rewind (inFile);
    data_buf = (unsigned char*) malloc (sizeof(char)*lSize);
    fread (data_buf, 1, lSize, inFile);
    if(inFile != NULL)
        fclose(inFile);
        
    in_buf = (struct autok_single_element *)data_buf;
    nv_data = (ap_nvram_sdio_config_struct*)g_nv_buf;   
    file_count = nv_data->file_count;
    
    // file update
    for(i=0; i<file_count; i++){
        if(nv_data->id[i] == id){
            struct autok_single_element *autok_ptr = (struct autok_single_element*)&nv_data->data[i*EACH_FILE_SIZE];
            if(autok_ptr->voltage == in_buf->voltage){
                LOGI("file_update:%d", file_count);
                nv_data->file_length[i] = lSize;
                memcpy(autok_ptr, data_buf, lSize);
                write_nvram(data_buf, lSize, id, i, file_count);
                break;
            }
        }
    }
    
    // new file(no found in table)
    if(i==file_count){
        //LOGI("add new file with size:%l", lSize);
        nv_data->id[file_count] = id;
        nv_data->file_length[file_count] = lSize;
        if(file_count + lSize > (sizeof(nv_data->data)/sizeof(nv_data->data[0])))
            lSize = sizeof(nv_data->data)/sizeof(nv_data->data[0]) - file_count;
        memcpy(&nv_data->data[file_count], data_buf, lSize);        
        write_nvram(data_buf, lSize, id, file_count, file_count+1);
        nv_data->file_count += 1;   // zero based   
    }    
    AUTOK_FREE(data_buf);
    return 0;
}

int is_nvram_data_exist(int id, unsigned int voltage)
{
    int file_count;
    int i;
    ap_nvram_sdio_config_struct *nv_data = (ap_nvram_sdio_config_struct*)g_nv_buf;   
    file_count = nv_data->file_count;
    LOGI("file_count[%d]\n", file_count);
    for(i=0; i<file_count; i++){
        if(nv_data->id[i] == id){
            struct autok_single_element autok_ptr;;
            char *ptr = &nv_data->data[i*EACH_FILE_SIZE];
            memcpy(&autok_ptr.voltage, ptr+2, 4);
            if(autok_ptr.voltage == voltage){
                return 1;
            }
        }
    }
    return 0;
}

std::list<unsigned int> get_nvram_voltages(int id)
{
    int file_count;
    int i;
    std::list<unsigned int> vol_list;
    ap_nvram_sdio_config_struct *nv_data = (ap_nvram_sdio_config_struct*)g_nv_buf;   
    file_count = nv_data->file_count;
    LOGI("file_count[%d]\n", file_count);
    for(i=0; i<file_count; i++){
        if(nv_data->id[i] == id){ 
            struct autok_single_element autok_ptr;;
            char *ptr = &nv_data->data[i*EACH_FILE_SIZE];
            memcpy(&autok_ptr.voltage, ptr+2, 4);
            vol_list.push_back(autok_ptr.voltage);
        }
    }
    return vol_list;
}

int get_nvram_param_count(int id)
{
    int file_count;
    int i;
    int param_count = 0;
    ap_nvram_sdio_config_struct *nv_data = (ap_nvram_sdio_config_struct*)g_nv_buf;   
    file_count = nv_data->file_count;
    LOGI("file_count[%d]\n", file_count);
    for(i=0; i<file_count; i++){
        if(nv_data->id[i] == id){
            char *ptr = &nv_data->data[i*EACH_FILE_SIZE];
            memcpy(&param_count, ptr+1, 1);
            return param_count;
        }
    }    
    return 0;
}

int init_autok_nvram()
{
    int result = 0;
    g_nv_buf = (unsigned char*)malloc(sizeof(ap_nvram_sdio_config_struct));
    if((result = sdio_read_nvram(g_nv_buf))!=0)
        return result;
    return result;
}

int close_nvram()
{
    AUTOK_FREE(g_nv_buf);
    return 0;
}
