
// 陀螺仪直接使用逐飞的init函数即可进行初始化，但逐飞添加了while函数，可能需要重试保证完全初始化完成
	while(1)
		{
        if(imu_init())
            printf("\r\nIMU660RA init error.");      // IMU660RA 初始化失败
        else
            break;
        gpio_toggle_level(LED_DEBUG);                     // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
	// while初始化时间可能较长

//使用imu_update函数获取数据
imu_update();