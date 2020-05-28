# M031BSP_PWM_1Hz_HXT_4M
 M031BSP_PWM_1Hz_HXT_4M

update @ 2020/05/28

- Enable HXT configuration , with crystal 4MHz

in system_M031Series.h , modify __HXT

{

#define __HXT       (4000000UL) 

}

in SYS_Init @ main.c , enable HXT function ,

{

    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);	
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

}

- PWM0_CH4 (PB.1) output 1Hz 

![image](https://github.com/released/M031BSP_PWM_1Hz/blob/master/FREQ_1Hz_RESOLUTION_1K.jpg)

- enable CKO (PB.14) to monitor clock output
