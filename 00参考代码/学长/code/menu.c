/*
 * menu.c
 *
 *  Created on: 2025年5月11日
 *      Author: 26672
 */
#include "zf_common_headfile.h"

char testnumchar=0;
int  testnumint=0;
float testnumfloat=0;
int16 testnumshort=0;
//用于存取的数组
uint8 data_buff[255]={0};

Menu_var var1[]={//二级列表1选项下的内容，PID
                 //参数为：名称，参数地址，单次操作修改量，数据类型

        {"Run",&Run,1,int161},
        {"SPEED_BASE",&SPEED_BASE,10,int161},
        {"speed_extra",&speed_extra1,10,int161},
        {"Turn_p",&Turn_p,1,float1},
        {"Turn_p2",&Turn_p2,0.1,float1},
        {"Turn_d",&Turn_d,1,float1},
        {"Turn_gkd",&Turn_gkd,0.1,float1},

        {"z_p",&z_p,1,float1},
        {"z_d",&z_d,0.1,float1},

        {"fz_p_base",&fz_p_base,0.01,float1},
};

Menu_var var2[]={//二级列表1选项下的内容，DBBridge
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"no 0, 3cm 2, 5cm 1",&testnumchar,1,char1},
        {"DB_type[0]",&DB_type[0],1,int1},
        {"DB_type[1]",&DB_type[1],1,int1},
        {"DB_type[2]",&DB_type[2],1,int1},
        {"DB_speed",&DBBridge_speed,10,int1},
        {"DB_after_speed",&DBBridge_after_speed,10,int1},
        {"DB_stop_speed",&DBBridge_stop_speed,50,int1},
        {"db_stop_time",&dbbridge_stop_time,100,int1},
        {"db_stop_time3",&dbbridge_stop_time3,100,int1},
        {"db_pass_time",&dbbridge_pass_time,100,int1},
        {"db_pass_time3",&dbbridge_pass_time3,100,int1},
};

Menu_var var3[]={//二级列表1选项下的内容，jump
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"jump_ready",&jump_ready,1,int1},
        {"jump_linelength",&jump_linelength,1,int1},
        {"jump_wait",&jump_wait,1,int1},
        {"street_len_40",&street_len_40,0,int1},
        {"vs_jump",&vs_jump,1,int1},
        {"jump_after_speed",&jump_after_speed,1,int1}
};

Menu_var var4[]={//二级列表1选项下的内容，//Ring
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"Ring_mode",&Ring_mode,1,int1},
        {"ring_speed",&ring_speed,10,int1},
        {"Ring_type[0]",&Ring_type[0],1,int1},
        {"Ring_type[1]",&Ring_type[1],1,int1},
        {"Ring_type[2]",&Ring_type[2],1,int1},
        {"Ring_type[3]",&Ring_type[3],1,int1},
        {"Ring_type[4]",&Ring_type[4],1,int1}
};
Menu_var var5[]={//二级列表1选项下的内容，//RRing2
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"Rsmall_dis21",&Rsmall_ring_dis21,50,int1},
        {"Rbig_dis21",&Rbig_ring_dis21,50,int1},

        {"Rsmall_ang21",&Rsmall_ring_ang21,10,int1},
        {"Rsmall_ang22",&Rsmall_ring_ang22,10,int1},
        {"Rsmall_ang23",&Rsmall_ring_ang23,10,int1},
        {"Rsmall_dis22",&Rsmall_ring_dis22,50,int1},
        {"Rring_errmax",&Rring_errmax,1,int1},
        {"Rbig_ang21",&Rbig_ring_ang21,10,int1},
        {"Rbig_ang22",&Rbig_ring_ang22,10,int1},
        {"Rbig_ang23",&Rbig_ring_ang23,10,int1},
        {"Rbig_dis22",&Rbig_ring_dis22,50,int1},
};
Menu_var var6[]={//二级列表1选项下的内容，//LRing2
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"Lsmall_dis21",&Lsmall_ring_dis21,50,int1},
        {"Lbig_dis21",&Lbig_ring_dis21,50,int1},
        {"Lsmall_ang21",&Lsmall_ring_ang21,10,int1},
        {"Lsmall_ang22",&Lsmall_ring_ang22,10,int1},
        {"Lsmall_ang23",&Lsmall_ring_ang23,10,int1},
        {"Lsmall_dis22",&Lsmall_ring_dis22,50,int1},
        {"Lring_errmax",&Lring_errmax,1,int1},
        {"Lbig_ang21",&Lbig_ring_ang21,10,int1},
        {"Lbig_ang22",&Lbig_ring_ang22,10,int1},
        {"Lbig_ang23",&Lbig_ring_ang23,10,int1},
        {"Lbig_dis22",&Lbig_ring_dis22,50,int1},
};
Menu_var var7[]={//二级列表1选项下的内容，//ramp
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"ramp_ready",&ramp_ready,1,int1},
        {"ramp_speed",&ramp_speed,10,int1},
        {"ramp_after_speed",&ramp_after_speed,10,int1},
        {"ramp_length[0]",&ramp_length[0],100,int1},
        {"ramp_length[1]",&ramp_length[1],100,int1},
        {"ramp_length[2]",&ramp_length[2],100,int1},
        {"ramp_length[3]",&ramp_length[3],100,int1},
        {"testnumfloat",&testnumfloat,0.01,float1}
};
Menu_var var8[]={//二级列表1选项下的内容，//after_speed
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"ramp_after_speed",&ramp_after_speed,10,int1},
        {"ring_after_speed",&ring_after_speed,10,int1},
        {"jump_after_speed",&jump_after_speed,10,int1},
        {"DB_after_speed",&DBBridge_after_speed,10,int1},

        {"lz_after_speed",&luzhang_after_speed,10,int1},
        {"testnumfloat",&testnumfloat,0.01,float1}
};
Menu_var var9[]={//二级列表1选项下的内容，//luzhang
                 //参数为：名称，参数地址，单次操作修改量，数据类型
        {"l 1,r 2,s 3,r 0",&testnumchar,1,char1},
        {"luzhang_types[0]",&luzhang_types[0],1,int1},
        {"luzhang_types[1]",&luzhang_types[1],1,int1},
        {"luzhang_types[2]",&luzhang_types[2],1,int1},
        {"luzhang_types[3]",&luzhang_types[3],1,int1},
        {"luzhang_types[4]",&luzhang_types[4],1,int1},
        {"lz_after_speed",&luzhang_after_speed,10,int1},
        {"lz_no_dis",&lz_no_dis,10,int1},
        {"testnumfloat",&testnumfloat,0.01,float1}
};
Menu_var option[]={//一级列表,参数为：名字，表头，第三位固定为0，第四位固定为no
        {"PID",var1,0,no},
        {"DBBridge",var2,0,no},
        {"Jump",var3,0,no},
        {"Ring",var4,0,no},
        {"RRing2",var5,0,no},
        {"LRing2",var6,0,no},
        {"ramp",var7,0,no},
        {"after_speed",var8,0,no},
        {"luzhang",var9,0,no}

};

uint8 var_len[] = {//列表长度
        ///////////////这里向下，添加一级菜单新的分支
        sizeof(var1)/sizeof(Menu_var),
        sizeof(var2)/sizeof(Menu_var),
        sizeof(var3)/sizeof(Menu_var),
        sizeof(var4)/sizeof(Menu_var),
        sizeof(var5)/sizeof(Menu_var),
        sizeof(var6)/sizeof(Menu_var),
        sizeof(var7)/sizeof(Menu_var),
        sizeof(var8)/sizeof(Menu_var),
        sizeof(var9)/sizeof(Menu_var),
        ///////////////这里往上，添加一级菜单新的分支
        sizeof(option)/sizeof(Menu_var)
                   };
uint8 len_var_len = sizeof(var_len)/sizeof(uint8);//一级列表长度，不用管

int16 float_save;//存储float类型的中间变量

void var_save(){//这里在参数存储前做不同处理
    data_buff[0] = testnumchar;

    data_buff[1] = testnumint;
    data_buff[2] = (testnumint>>8);

    float_save   = testnumfloat*100;
    //两位只能存储0-65535，这里乘以100，也就是0-655.35根据不同范围乘以不同的数
    data_buff[3] = float_save;
    data_buff[4] = (float_save>>8);

    data_buff[5] = jump_wait;
    data_buff[6] = Ring_type[0];
    data_buff[7] = Ring_type[1];
    data_buff[8] = Ring_type[2];
    data_buff[9] = Ring_type[3];
    data_buff[10] = Ring_type[4];
    data_buff[11] = vs_jump;
    data_buff[12] = DB_type[0];
    data_buff[13] = DB_type[1];
    data_buff[14] = DB_type[2];
    data_buff[15] = luzhang_types[0];
    data_buff[16] = luzhang_types[1];
    data_buff[17] = luzhang_types[2];
    data_buff[18] = luzhang_types[3];
    data_buff[19] = luzhang_types[4];
    data_buff[20] =  ring_speed;
    data_buff[21] =  (ring_speed>>8);
    data_buff[22] =  speed_extra1;
    data_buff[23] =  jump_ready;
    data_buff[24] =  ramp_ready;
    at24c02_write_bytes(0,data_buff,255);


}
void var_load(){//这里在参数读出后做不同处理
    at24c02_read_bytes(0,data_buff,256);
    testnumchar = data_buff[0];
    testnumint = data_buff[1]+(data_buff[2]<<8);
    testnumfloat = (data_buff[3]+(data_buff[4]<<8))/100.0;
    Ring_type[0]=data_buff[6];
    Ring_type[1]=data_buff[7];
    Ring_type[2]=data_buff[8];
    Ring_type[3]=data_buff[9];
    Ring_type[4]=data_buff[10];
    vs_jump     =data_buff[11];
    DB_type[2]  =data_buff[14];
    DB_type[1]  =data_buff[13];
    DB_type[0]  =data_buff[12];
    luzhang_types[0]=data_buff[15];
    luzhang_types[1]=data_buff[16];
    luzhang_types[2]=data_buff[17];
    luzhang_types[3]=data_buff[18];
    luzhang_types[4]=data_buff[19];
    ring_speed      =data_buff[20]+(data_buff[21]<<8);
    speed_extra1    =data_buff[22];
    jump_ready      =data_buff[23];
    ramp_ready      =data_buff[24];
}
