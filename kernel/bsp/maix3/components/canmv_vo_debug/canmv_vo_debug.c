#include <rtthread.h>
#include <rthw.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <ioremap.h>
#include "riscv_mmu.h"

#define VO_MEMORY_ADDR_START 0x90840000
#define VO_MEMORY_READ_LEN 0x1000
#define VO_MEMORY_SIZE 0x400
#define MAP_SIZE    PAGE_SIZE
#define MAP_MASK    (MAP_SIZE - 1)
#define BITS_PER_LONG                                               64
#define BIT_MASK(nr)                                                (1ul << ((nr) % BITS_PER_LONG))

static void print_video_layer_info(int i, const rt_uint32_t read_result[]){
    int j = 0;

    rt_kprintf("[vo layer%d]:\t", i);

    if((read_result[0x118/4] >> i) & 0x1){
        rt_kprintf("enabled\n");
    }
    else{
        rt_kprintf("disabled\n");
        return;
    }

    if(i == 1){
        if(read_result[0xA20/4] & 0x1){
            rt_kprintf("ctl:enabled\n");
        }else{
            rt_kprintf("ctl:disabled\n"); 
            return;
        }

        rt_kprintf("rotate: %d\t", (read_result[0xa20/4] >> 4 & 0x3)*90);

        if(read_result[0xa20/4] >> 6 & 0x1)
            rt_kprintf("x mirror\t");

        if(read_result[0xa20/4] >> 7 & 0x1)
            rt_kprintf("y mirror\t");

        rt_kprintf("\n");

        if((read_result[0xa20/4] >> 8 & 0x1) == 0){
            rt_kprintf("format: YUV420\t");
            rt_kprintf("stride (y:uv 0x%x:0x%x)\n", read_result[(0xa28)/4] & 0xFFFF, (read_result[(0xa28)/4] >> 16) & 0xFFFF);
        }
        else
            rt_kprintf("format: YUV422\n");

        rt_kprintf("position (x:y:w:h 0x%x:0x%x:0x%x:0x%x)\n", read_result[(0x0C9)/4] & 0xFFF, (read_result[(0x0C9)/4] >> 16) & 0xFFF, read_result[(0xa24)/4] & 0xFFF, (read_result[(0xa24)/4] >> 16) & 0xFFF);
        rt_kprintf("buffer0 (Y:UV 0x%x:0x%x)\n", read_result[(0xa2c)/4], read_result[(0xa30)/4]);
        rt_kprintf("buffer1 (Y:UV 0x%x:0x%x)\n", read_result[(0xa34)/4], read_result[(0xa38)/4]);
        return;
    }

    j = i - 2;

    if(read_result[(0x200+j*0x40)/4] & 0x1 == 0){
        rt_kprintf("ctl:disabled\n");
        return;
    }

    rt_kprintf("ctl:enabled\n");

    if((read_result[(0x200+j*0x40)/4] >> 4)& 0x1){
        rt_kprintf("format:YUV422\n");
    }
    else if((read_result[(0x200+j*0x40)/4] >> 8)& 0x1){
        rt_kprintf("format:YUV420\n");
    }
    else{
        rt_kprintf("format:unknown format\n");
    }

    rt_kprintf("buffer0 (Y:UV 0x%x:0x%x)\n", read_result[(0x204+j*0x40)/4], read_result[(0x208+j*0x40)/4]);
    rt_kprintf("buffer1 (Y:UV 0x%x:0x%x)\n", read_result[(0x20C+j*0x40)/4], read_result[(0x210+j*0x40)/4]);
    rt_kprintf("position (x:y:w:h 0x%x:0x%x:0x%x:0x%x)\n", read_result[(0x214+j*0x40)/4] & 0xFFF, (read_result[(0x214+j*0x40)/4] >> 16) & 0xFFF, read_result[(0x220+j*0x40)/4] & 0xFFF, (read_result[(0x220+j*0x40)/4] >> 16) & 0xFFF);
}

static void print_osd_layer_info(int i, const rt_uint32_t read_result[]){
    int j = i+4;
    rt_kprintf("[osd%d]:\t", i);
    if((read_result[0x118/4] >> j) & 0x1){
        rt_kprintf("enabled\n");
    }
    else{
        rt_kprintf("disabled\n");
        return;
    }

    switch(read_result[(0x280 + i*0x40)/4]&0xF)
    {
        case 0:
            rt_kprintf("format: RGB-24bit\t");
            break;
        case 1:
            rt_kprintf("format: Monochrome-8-bit\t");
            break;
        case 2:
            rt_kprintf("format: RGB565\t");
            break;
        case 3:
            rt_kprintf("format: ARGB8888\t");
            break;
        case 4:
            rt_kprintf("format: ARGB4444\t");
            break;
        case 5:
            rt_kprintf("format: ARGB1555\t");
            break;

    }
    switch((read_result[(0x280 + i*0x40)] >> 4)&0x7)
    {
        case 0:
            rt_kprintf("ALPHA: fixed value\n");
            break;
        case 1:
            rt_kprintf("ALPHA: Alpha block\n");
            break;
        case 2:
            rt_kprintf("ALPHA: R channel\n");
            break;
        case 3:
            rt_kprintf("ALPHA: G channel\n");
            break;
        case 4:
            rt_kprintf("ALPHA: B channel\n");
            break;
        case 5:
            rt_kprintf("ALPHA: Alpha Channel(when format is ARGB8888/ARGB4444/ARBG1555)\n");
            break;
    }

    rt_kprintf("buffer0 (DATA:ALPAH 0x%x:0x%x)\n", read_result[(0x288+i*0x40)/4], read_result[(0x28C+i*0x40)/4]);
    rt_kprintf("buffer1 (DATA:ALPAH 0x%x:0x%x)\n", read_result[(0x290+i*0x40)/4], read_result[(0x294+i*0x40)/4]);
    rt_kprintf("stride (DATA:ALPAH 0x%x:0x%x)\n", read_result[(0x29C+i*0x40)/4]&0xFFFF, (read_result[(0x29C+i*0x40)/4] >> 16)&0xFFFF);
    rt_kprintf("position (x:y:w:h 0x%x:0x%x:0x%x:0x%x)\n", read_result[(0xE8+i*8)/4] & 0xFFF, (read_result[(0x2EC+i*8)/4] >> 16) & 0xFFF, read_result[(0x284+i*0x40)/4] & 0xFFFF, (read_result[(0x284+i*0x40)/4] >> 16) & 0xFFFF);

}

static int query_vo_status(void){
    int fd;
    volatile void *virt_addr = RT_NULL;
    void *map_base = RT_NULL;
    volatile rt_uint32_t read_result[VO_MEMORY_READ_LEN/4];
    rt_ubase_t target;


    map_base = rt_ioremap_nocache((void *)(VO_MEMORY_ADDR_START & ~VO_MEMORY_READ_LEN), VO_MEMORY_READ_LEN);
    memcpy(read_result, map_base, sizeof(read_result));
    rt_iounmap(map_base);
    rt_kprintf("============= canmv_vo_status =========================\n"); 
    rt_kprintf("[vo setting]:\n");
    rt_kprintf("VO_DISP_ZONE(x: %d~%d, y: %d~%d)\t", (read_result[0xC0/4] & 0x1FFF),((read_result[0xC0/4] >> 16) & 0x1FFF),(read_result[0xC4/4] & 0x1FFF),((read_result[0xC4/4] >> 16) & 0x1FFF));
    rt_kprintf("VO_DISP_SIZE(h: %d, v: %d)\n", (read_result[0x11C/4] & 0xFFF),((read_result[0x11C/4] >> 16) & 0xFFF));
    rt_kprintf("VO_DISP_SIZE(h: %d, v: %d)\n", (read_result[0x11C/4] & 0xFFF),((read_result[0x11C/4] >> 16) & 0xFFF));
    rt_kprintf("VO_DISP_BACKGROUND(YUV:0x%x)\n", (read_result[0x3D0/4] & 0xFFFFFF));
    
    for(int i=1; i<4; i++){
        print_video_layer_info(i, read_result);
    }

    for(int i=0; i<8; i++){
        print_osd_layer_info(i, read_result);
    }    
    
    return 0;
}

int save_data_to_file(void *dump_address, size_t dump_size, const char *filename)
{
    FILE *fp;
    size_t written;
    rt_uint8_t *data = (rt_uint8_t *)dump_address;
    
    if (dump_address == RT_NULL || filename == RT_NULL) {
        return -1;
    }
    
    // 打开文件
    fp = fopen(filename, "w");
    if (fp == NULL) {
        rt_kprintf("Error: 无法打开文件 %s\n", filename);
        return -2;
    }
    fwrite(dump_address, 1, dump_size, fp);
    fclose(fp);
    rt_kprintf("成功保存0x%x %d 字节数据到文件 %s\n", dump_address, dump_size, filename);
    return 0;
}

static void dump_buffer(const char *cmd){
    char param[10] = {0};
    int layer_num = 0;
    int dump_address = 0;
    int dump_size = 0;
    int osd_num = 0;
    char file_save_name[30] = {0}; 
    int fd;
    volatile void *virt_addr = RT_NULL;
    void *map_base = RT_NULL;
    volatile rt_uint32_t read_result[VO_MEMORY_READ_LEN/4];
    rt_ubase_t target;


    map_base = rt_ioremap_nocache((void *)(VO_MEMORY_ADDR_START & ~VO_MEMORY_READ_LEN), VO_MEMORY_READ_LEN);
    memcpy(read_result, map_base, sizeof(read_result));
    rt_iounmap(map_base);

    if (sscanf(cmd, "dump %s", param) == 1) {
        rt_kprintf("dump %s buffer\n", param); // 实际应用中查询真实值
    } else {
        rt_kprintf("格式错误, 正确格式:dump layer1~3/osd0~6\n");
        return;
    }

    if (strcmp(param, "layer1") == 0){
        if(!((read_result[0x118/4] >> 1) & 0x1) || !(read_result[0xA20/4] & 0x1)){
            rt_kprintf("layer1 is not enabled\n");
            return;
        }
        dump_address = read_result[(0xa2c)/4];
        if((read_result[0xa20/4] >> 8 & 0x1) == 0)
            dump_size = 1.5*(read_result[(0xa24)/4] & 0xFFF)*((read_result[(0xa24)/4] >> 16) & 0xFFF);
        else
            dump_size = 2*(read_result[(0xa24)/4] & 0xFFF)*((read_result[(0xa24)/4] >> 16) & 0xFFF);
    }
    else if (strcmp(param, "layer2") == 0){
        if(!((read_result[0x118/4] >> 2) & 0x1)|| !(read_result[0x200/4] & 0x1)){
            rt_kprintf("layer2 is not enabled\n");
            return;
        }
        dump_address = read_result[(0x204)/4];
        if((read_result[0x200/4] >> 8 & 0x1) == 0)
            dump_size = 1.5*(read_result[(0x214)/4] & 0xFFF)*((read_result[(0x214)/4] >> 16) & 0xFFF);
        else
            dump_size = 2*(read_result[(0x214)/4] & 0xFFF)*((read_result[(0x214)/4] >> 16) & 0xFFF);
    }
    else if(strcmp(param, "layer3") == 0){
        if(!((read_result[0x118/4] >> 3) & 0x1) || !(read_result[0x240/4] & 0x1)){
            rt_kprintf("layer3 is not enabled\n");
            return;
        }
        dump_address = read_result[(0x244)/4];
        if((read_result[0x240/4] >> 8 & 0x1) == 0)
            dump_size = 1.5*(read_result[(0x254)/4] & 0xFFF)*((read_result[(0x254)/4] >> 16) & 0xFFF);
        else
            dump_size = 2*(read_result[(0x254)/4] & 0xFFF)*((read_result[(0x254)/4] >> 16) & 0xFFF);
    }
    else if (strstr(param, "osd") != NULL){
        if (sscanf(param, "%*[^0-9]%d", &osd_num) == 1){
            if(osd_num < 0 || osd_num > 7){
                rt_kprintf("osd num need in 0~7\n");
                return;
            }
        }

        if(!(read_result[0x118/4] >> (osd_num+4)) & 0x1){
            rt_kprintf("%s is not enabled\n", param);
            return;
        }
        dump_address = read_result[(0x288+osd_num*0x40)/4];
        dump_size = (read_result[(0x29C+osd_num*0x40)/4]&0xFFFF) * ((read_result[(0x284+osd_num*0x40)/4] >> 16) & 0xFFFF) * 8;
    }

    if(dump_address != 0){
        sprintf(file_save_name, "./dump_%s.txt", param);
        save_data_to_file(dump_address, dump_size, file_save_name);
    }
    else{
        rt_kprintf("格式错误, 正确格式:dump layer1~3/osd0~6\n");
    }
}

static void set_layer_onoff(const char *cmd){
    char param[10] = {0};
    int param2 = 0;
    void *map_base = RT_NULL;
    int bit_num = 0;
    volatile rt_uint32_t read_result[VO_MEMORY_READ_LEN/4];
    volatile rt_uint64_t writeval;
    volatile void *virt_addr = RT_NULL;

    if (sscanf(cmd, "set %9s %d", param, &param2) == 2) {
        rt_kprintf("set %s %d\n", param, param2); // 实际应用中查询真实值
    } else {
        rt_kprintf("格式错误, 正确格式:set [layer1~3/osd0~6] [0/1]\n");
        return;
    }

    map_base = rt_ioremap_nocache((void *)(VO_MEMORY_ADDR_START & ~VO_MEMORY_READ_LEN), VO_MEMORY_READ_LEN);
    memcpy(read_result, map_base, sizeof(read_result));
    

    if(strcmp(param, "layer1") == 0)
        bit_num = 1;
    else if(strcmp(param, "layer2") == 0)
        bit_num = 2;
    else if(strcmp(param, "layer3") == 0)
        bit_num = 3;
    else if(strcmp(param, "osd0") == 0)
        bit_num = 4;
    else if(strcmp(param, "osd1") == 0)
        bit_num = 5;
    else if(strcmp(param, "osd2") == 0)
        bit_num = 6;
    else if(strcmp(param, "osd3") == 0)
        bit_num = 7;
    else if(strcmp(param, "osd4") == 0)
        bit_num = 8;
    else if(strcmp(param, "osd5") == 0)
        bit_num = 9;
    else if(strcmp(param, "osd6") == 0)
        bit_num = 10;
    else if(strcmp(param, "osd7") == 0)
        bit_num = 11;

        
    if((!((read_result[0x118/4] >> bit_num) & 0x1)) && (param2==0)){
        rt_kprintf("%s is already disabled\n", param);
        rt_iounmap(map_base);
        return;
    }

    if(((read_result[0x118/4] >> bit_num) & 0x1) && (param2==1)){
        rt_kprintf("%s is already enabled\n", param);
        rt_iounmap(map_base);
        return;
    }

    virt_addr = map_base + 0x118;
    writeval = (read_result[0x118/4] & ~(BIT_MASK(bit_num))) | (param2 << bit_num);
     *((rt_uint8_t *) virt_addr) = writeval;
    rt_kprintf("set 0x%x 0x%x\n", virt_addr, writeval);
    virt_addr = map_base + 0x4;
    *((rt_uint8_t *) virt_addr) = 0x11;
    rt_kprintf("set 0x%x 0x11\n", virt_addr);
    rt_iounmap(map_base);
    return;
}

static void handle_command(const char *cmd) {
    if (strstr(cmd, "q") == cmd) {
        query_vo_status();
        return;
    }

    if (strstr(cmd, "dump") == cmd) {
        dump_buffer(cmd);
        return;
    }

    if (strstr(cmd, "set") == cmd) {
        set_layer_onoff(cmd);
        return;
    }
}

int canmv_vo_debug(int argc, char **argv) {
    char cmd[50] = {0};

    rt_kprintf("============= canmv_vo_debug =========================\n");
    rt_kprintf("支持命令：\n");
    rt_kprintf("  q                   - 查询vo状态\n");
    rt_kprintf("  set <参数名> <数值>  - 打开/关闭某一层layer或者OSD(例如:set osd0 0/1)\n");
    rt_kprintf("  dump <参数名>        - 保存layer的数据(例如:dump layer1~3/osd0~7)\n");
    rt_kprintf("  exit                 - 退出命令\n");
    rt_kprintf("请输入命令：\n");

    while(1){
        memset(cmd, 0, sizeof(cmd));
        if (fgets(cmd, sizeof(cmd), stdin) == NULL) {
            rt_kprintf("输入读取失败，重试...\n");
            rt_thread_mdelay(1000);
            continue;
        }
        cmd[strcspn(cmd, "\r\n")] = '\0';

        if (strlen(cmd) > 0) {  // 输入不为空时才显示
            rt_kprintf("你输入的命令：%s\n", cmd);  // 直接通过rt_kprintf输出到串口
        }

        if (cmd == NULL || strlen(cmd) == 0) {
            rt_kprintf("输入为空，请重新输入\n");
            continue;
        }  

        if (strcmp(cmd, "exit") == 0) {
            rt_kprintf("退出\n");
            return 0;
        }
        
        handle_command(cmd);
    }

    

    //target = strtoul(argv[1], 0, 0);
    //if(target & 0x3) {
    //    rt_kprintf("The address must be 8-byte aligned!\n");
    //    return -1;
    //}

    //if(argc > 2)
    //    access_type = tolower(argv[2][0]);

    
    return 0;
}

MSH_CMD_EXPORT(canmv_vo_debug, Simple program to debug vo module);