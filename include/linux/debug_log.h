/*
 * debug_log.h
 *
 * debug log Layer
 *
 * Copyright (C) 2011 Huawei Device. All rights reserved.
 *
 * Author: yangjun y00185015 <june.yang@huawei.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

 /******************************************************************************
   Copyright (C), 2011-2015, Huawei Tech. Co., Ltd.
   File name:     debug_log.h
   Author:       Version:        Date:
   y00185015     1.0             20110730
   Description:  Kernel Log模块打印头文件
   Function:     比DBG_CONSOLE_LOGLEVEL_MIN级别高且比DBG_CONSOLE_LOGLEVEL_MAX级别小的消息才会打印；
   Others:
 ******************************************************************************/


    //#define DBG_CONSOLE_LOGLEVEL_MIN    0   /*-1~10, 打印最小级别，高于等于此级别的消息才有可能打印；*/
    //#define DBG_CONSOLE_LOGLEVEL_MAX   -1    /*-1~10, 打印最大级别，低于等于此级别的消息才有可能打印；*/

    //#define DEV_DBG_CONSOLE_LOGLEVEL_MIN    0   /*-1~10, 打印最小级别，高于等于此级别的消息才有可能打印；*/
    //#define DEV_DBG_CONSOLE_LOGLEVEL_MAX   -1    /*-1~10, 打印最大级别，低于等于此级别的消息才有可能打印；*/
//#include <linux/debug_log.h>


//#define DEBUG_LOG_ENABLE  //#y00185015#20110726#Add#Add begin for debug_log#

#ifndef DBG_CONSOLE_LOGLEVEL_MIN
#define DBG_CONSOLE_LOGLEVEL_MIN    0   /*-1~10, 打印最小级别，高于等于此级别的消息才有可能打印；*/
#endif  //#ifndef DBG_CONSOLE_LOGLEVEL_MIN
#ifndef DBG_CONSOLE_LOGLEVEL_MAX
#define DBG_CONSOLE_LOGLEVEL_MAX   -1    /*-1~10, 打印最大级别，低于等于此级别的消息才有可能打印；*/
#endif  //#ifndef DBG_CONSOLE_LOGLEVEL_MAX

#ifndef DEV_DBG_CONSOLE_LOGLEVEL_MIN
#define DEV_DBG_CONSOLE_LOGLEVEL_MIN    0   /*-1~10, 打印最小级别，高于等于此级别的消息才有可能打印；*/
#endif  //#ifndef DEV_DBG_CONSOLE_LOGLEVEL_MIN
#ifndef DEV_DBG_CONSOLE_LOGLEVEL_MAX
#define DEV_DBG_CONSOLE_LOGLEVEL_MAX   -1    /*-1~10, 打印最大级别，低于等于此级别的消息才有可能打印；*/
#endif  //#ifndef DEV_DBG_CONSOLE_LOGLEVEL_MAX


#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__


//#y00185015#20110730#printk#
#if DBG_CONSOLE_LOGLEVEL_MIN<=0 && 0<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_EMERG "<0>" /* system is unusable */
    #define dpr_0     printk
    #define if_dpr_0(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_0(...)    {}
    #define if_dpr_0(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=1 && 1<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_ALERT "<1>" /* action must be taken immediately */
    #define dpr_1     printk
    #define if_dpr_1(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_1(...)    {}
    #define if_dpr_1(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=2 && 2<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_CRIT "<2>" /* critical conditions */
    #define dpr_2      printk
    #define if_dpr_2(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_2(...)    {}
    #define if_dpr_2(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=3 && 3<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_ERR "<3>" /* error conditions */
    #define dpr_3       printk
    #define if_dpr_3(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_3(...)    {}
    #define if_dpr_3(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=4 && 4<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_WARNING	"<4>" /* warning conditions */
    #define dpr_4   printk
    #define if_dpr_4(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_4(...)    {}
    #define if_dpr_4(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=5 && 5<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_NOTICE "<5>" /* normal but significant condition */
    #define dpr_5    printk
    #define if_dpr_5(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_5(...)    {}
    #define if_dpr_5(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=6 && 6<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_INFO	"<6>" /* informationa */
    #define dpr_6      printk
    #define if_dpr_6(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_6(...)    {}
    #define if_dpr_6(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=7 && 7<=DBG_CONSOLE_LOGLEVEL_MAX // KERN_DEBUG "<7>" /* debug-level messages */
    #define dpr_7      printk
    #define if_dpr_7(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_7(...)    {}
    #define if_dpr_7(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=8 && 8<=DBG_CONSOLE_LOGLEVEL_MAX //
    #define dpr_8      printk
    #define if_dpr_8(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_8(...)    {}
    #define if_dpr_8(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=9 && 9<=DBG_CONSOLE_LOGLEVEL_MAX //
    #define dpr_9      printk
#define if_dpr_9(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_9(...)    {}
    #define if_dpr_9(...)    { }
#endif

#if DBG_CONSOLE_LOGLEVEL_MIN<=10 && 10<=DBG_CONSOLE_LOGLEVEL_MAX //
    #define dpr_10      printk
    #define if_dpr_10(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dpr_10(...)    {}
    #define if_dpr_10(...)    { }
#endif

    #define dpr_yes   printk
    #define dpr_not(...)    {}
    #define dpr_dft   dpr_5
    #define dpr_tst   dpr_dft

    #define if_dpr_yes(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
    #define if_dpr_not(...)    {}
    #define if_dpr_dft   if_dpr_5
    #define if_dpr_tst   if_dpr_dft


//#y00185015#20110917#dev_printk#
#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=0 && 0<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_EMERG "<0>" /* system is unusable */
    #define dev_dpr_0     dev_printk
    #define if_dev_dpr_0(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_0(...)    {}
    #define if_dev_dpr_0(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=1 && 1<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_ALERT "<1>" /* action must be taken immediately */
    #define dev_dpr_1     dev_printk
    #define if_dev_dpr_1(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_1(...)    {}
    #define if_dev_dpr_1(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=2 && 2<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_CRIT "<2>" /* critical conditions */
    #define dev_dpr_2      dev_printk
    #define if_dev_dpr_2(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_2(...)    {}
    #define if_dev_dpr_2(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=3 && 3<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_ERR "<3>" /* error conditions */
    #define dev_dpr_3       dev_printk
    #define if_dev_dpr_3(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_3(...)    {}
    #define if_dev_dpr_3(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=4 && 4<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_WARNING	"<4>" /* warning conditions */
    #define dev_dpr_4   dev_printk
    #define if_dev_dpr_4(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_4(...)    {}
    #define if_dev_dpr_4(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=5 && 5<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_NOTICE "<5>" /* normal but significant condition */
    #define dev_dpr_5    dev_printk
    #define if_dev_dpr_5(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_5(...)    {}
    #define if_dev_dpr_5(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=6 && 6<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_INFO	"<6>" /* informationa */
    #define dev_dpr_6      dev_printk
    #define if_dev_dpr_6(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_6(...)    {}
    #define if_dev_dpr_6(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=7 && 7<=DEV_DBG_CONSOLE_LOGLEVEL_MAX // KERN_DEBUG "<7>" /* debug-level messages */
    #define dev_dpr_7      dev_printk
    #define if_dev_dpr_7(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_7(...)    {}
    #define if_dev_dpr_7(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=8 && 8<=DEV_DBG_CONSOLE_LOGLEVEL_MAX //
    #define dev_dpr_8      dev_printk
    #define if_dev_dpr_8(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_8(...)    {}
    #define if_dev_dpr_8(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=9 && 9<=DEV_DBG_CONSOLE_LOGLEVEL_MAX //
    #define dev_dpr_9      dev_printk
    #define if_dev_dpr_9(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_9(...)    {}
    #define if_dev_dpr_9(...)    { }
#endif

#if DEV_DBG_CONSOLE_LOGLEVEL_MIN<=10 && 10<=DEV_DBG_CONSOLE_LOGLEVEL_MAX //
    #define dev_dpr_10      dev_printk
    #define if_dev_dpr_10(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
#else
    #define dev_dpr_10(...)    {}
    #define if_dev_dpr_10(...)    { }
#endif

    #define dev_dpr_yes   printk
    #define dev_dpr_not(...)    {}
    #define dev_dpr_dft   dev_dpr_5
    #define dev_dpr_tst   dev_dpr_dft

    #define if_dev_dpr_yes(flag,...)     { if(flag)  { printk( __VA_ARGS__ ); } }
    #define if_dev_dpr_not(...)    {}
    #define if_dev_dpr_dft   if_dev_dpr_5
    #define if_dev_dpr_tst   if_dev_dpr_dft


#endif //#ifndef __DEBUG_LOG_H__
