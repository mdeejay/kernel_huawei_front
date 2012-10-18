/*
 * SiIxxxx <Firmware or Driver>
 *
 * Copyright (C) 2011 Silicon Image Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>

#include <mach/gpio.h>

//#include "sii_9244_driver.h"
#include "sii_9244_api.h"
#include "si_mhl_tx_api.h"
#include "si_mhl_defs.h"
//#include "si_timer_cfg.h"
#include "sii_reg_access.h"
#include "si_drv_mhl_tx.h"
#include <linux/i2c/sii_9244.h>




//interrupt mode or polling mode for 9244 driver
#define SiI9244DRIVER_INTERRUPT_MODE   1

//sbit pinMHLTxVbus_CTRL = P0^4;    // VDD 5V to MHL VBUS switch control

#define    APP_DEMO_RCP_SEND_KEY_CODE 0x41

bool_tt    vbusPowerState = true;        // false: 0 = vbus output on; true: 1 = vbus output off;


static bool mhl_initialized = false;

bool is_mhl_initialized(void)
{
	return mhl_initialized;
}

uint8_t PAGE_0_0X72;
uint8_t PAGE_1_0X7A;
uint8_t PAGE_2_0X92;
uint8_t PAGE_CBUS_0XC8;

static void init_PAGE_values(void)
{
    if (get_mhl_ci2ca_value())
    {
        PAGE_0_0X72 = 0x76;
        PAGE_1_0X7A = 0x7E;
        PAGE_2_0X92 = 0x96;
        PAGE_CBUS_0XC8 = 0xCC;
    }
    else
    {
        PAGE_0_0X72 = 0x72;
        PAGE_1_0X7A = 0x7A;
        PAGE_2_0X92 = 0x92;
        PAGE_CBUS_0XC8 = 0xC8;
    }

}

#if (VBUS_POWER_CHK == ENABLE)
///////////////////////////////////////////////////////////////////////////////
//
// AppVbusControl
//
// This function or macro is invoked from MhlTx driver to ask application to
// control the VBUS power. If powerOn is sent as non-zero, one should assume
// peer does not need power so quickly remove VBUS power.
//
// if value of "powerOn" is 0, then application must turn the VBUS power on
// within 50ms of this call to meet MHL specs timing.
//
// Application module must provide this function.
//
void    AppVbusControl( bool_tt powerOn )
{
    if( powerOn )
    {
        //pinMHLTxVbus_CTRL = 1;
        MHLSinkOrDonglePowerStatusCheck();
        TX_API_PRINT(("[MHL]App: Peer's POW bit is set. Turn the VBUS power OFF here.\n"));
    }
    else
    {
        //pinMHLTxVbus_CTRL = 0;
        TX_API_PRINT(("[MHL]App: Peer's POW bit is cleared. Turn the VBUS power ON here.\n"));
    }
}
#endif



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////////////////////////// Linux platform related //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//Debug test
#undef dev_info
#define dev_info dev_err
#define MHL_DRIVER_NAME "sii9244drv"

/***** public type definitions ***********************************************/

typedef struct {
    struct task_struct    *pTaskStruct;
    uint8_t                pendingEvent;        // event data wait for retrieval
    uint8_t                pendingEventData;    // by user mode application

} MHL_DRIVER_CONTEXT_T, *PMHL_DRIVER_CONTEXT_T;


/***** global variables ********************************************/

MHL_DRIVER_CONTEXT_T gDriverContext;

struct i2c_client *mhl_Sii9244_page0 = NULL;
struct i2c_client *mhl_Sii9244_page1 = NULL;
struct i2c_client *mhl_Sii9244_page2 = NULL;
struct i2c_client *mhl_Sii9244_cbus = NULL;

struct platform_data {
    void (*reset) (void);
};
static struct platform_data *Sii9244_plat_data;

//------------------------------------------------------------------------------
// Array of timer values
//------------------------------------------------------------------------------


uint16_t Int_count=0;

static bool_tt match_id(const struct i2c_device_id *id, const struct i2c_client *client)
{
    if (strcmp(client->name, id->name) == 0)
        return true;

    return false;
}

static bool_tt Sii9244_mhl_reset(void)
{
    Sii9244_plat_data = mhl_Sii9244_page0->dev.platform_data;
    if (Sii9244_plat_data->reset){
        Sii9244_plat_data->reset();
        return true;
    }
    return false;
}


/*****************************************************************************/
/**
 * @brief Wait for the specified number of milliseconds to elapse.
 *
 *****************************************************************************/
void HalTimerWait(uint16_t m_sec)
{
    unsigned long    time_usec = m_sec * 1000;

    usleep_range(time_usec, time_usec);
    //mdelay(m_sec);
}


// I2C functions used by the driver.
//
//------------------------------------------------------------------------------
uint8_t I2C_ReadByte(uint8_t SlaveAddr, uint8_t RegAddr)    //oscar
{
    uint8_t ReadData = 0;
#if 0
    switch (SlaveAddr)
    {
        case PAGE_0_0X72:
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_page0, RegAddr);
            break;
        case PAGE_1_0X7A:
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_page1, RegAddr);
            break;
        case PAGE_2_0X92:
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_page2, RegAddr);
            break;
        case PAGE_CBUS_0XC8:
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_cbus, RegAddr);
            break;
    }
    #endif
    if (SlaveAddr == PAGE_0_0X72)
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_page0, RegAddr);
    else if (SlaveAddr == PAGE_1_0X7A)
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_page1, RegAddr);
    else if (SlaveAddr == PAGE_2_0X92)
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_page2, RegAddr);
    else if (SlaveAddr == PAGE_CBUS_0XC8)
            ReadData = i2c_smbus_read_byte_data(mhl_Sii9244_cbus, RegAddr);

    return ReadData;
}

//------------------------------------------------------------------------------
void I2C_WriteByte(uint8_t SlaveAddr, uint8_t RegAddr, uint8_t Data)
{
#if 0
    switch (SlaveAddr)
    {
        case PAGE_0_0X72:
            i2c_smbus_write_byte_data(mhl_Sii9244_page0, RegAddr, Data);
            break;
        case PAGE_1_0X7A:
            i2c_smbus_write_byte_data(mhl_Sii9244_page1, RegAddr, Data);
            break;
        case PAGE_2_0X92:
            i2c_smbus_write_byte_data(mhl_Sii9244_page2, RegAddr, Data);
            break;
        case PAGE_CBUS_0XC8:
            i2c_smbus_write_byte_data(mhl_Sii9244_cbus, RegAddr, Data);
            break;
    }
#endif
    if (SlaveAddr == PAGE_0_0X72)
            i2c_smbus_write_byte_data(mhl_Sii9244_page0, RegAddr, Data);
    else if (SlaveAddr == PAGE_1_0X7A)
            i2c_smbus_write_byte_data(mhl_Sii9244_page1, RegAddr, Data);
    else if (SlaveAddr == PAGE_2_0X92)
            i2c_smbus_write_byte_data(mhl_Sii9244_page2, RegAddr, Data);
    else if (SlaveAddr == PAGE_CBUS_0XC8)
            i2c_smbus_write_byte_data(mhl_Sii9244_cbus, RegAddr, Data);
}
#ifdef SiI9244DRIVER_INTERRUPT_MODE

//------------------------------------------------------------------------------
static irqreturn_t Sii9244_mhl_interrupt(int irq, void *dev_id)
{
    //disable_irq_nosync(irq);

    //schedule_work(sii9244work);

    uint8_t Int_count=0;
    extern uint8_t    fwPowerState;

    //enable_irq(mhl_Sii9244_page0->irq);
    for(Int_count=0;Int_count<10;Int_count++){
        SiiMhlTxDeviceIsr();
        printk("Int_count=%d::::::::Sii9244 interrupt happened\n",Int_count);
        //SiiMhlTxGetEvents( &event, &eventParameter );
        #if 0
        if( MHL_TX_EVENT_NONE != event )
        {
            AppRcpDemo( event, eventParameter);
        }
        #endif
        if(POWER_STATE_D3 == fwPowerState)
                break;
    }
    //enable_irq(mhl_Sii9244_page0->irq);

    //printk("The sii9244 interrupt's top_half has been done and bottom_half will be processed..\n");
    //spin_unlock_irqrestore(&sii9244_lock, lock_flags);
    return IRQ_HANDLED;
}
#else
static int SiI9244_mhl_loop(void *nothing)
{
    //uint8_t    event;
    //uint8_t    eventParameter;

    printk("%s EventThread starting up\n", MHL_DRIVER_NAME);

       while (true)
        {
        /*
            Event loop
        */
        //
        // Look for any events that might have occurred.
        //
        //SiiMhlTxGetEvents( &event, &eventParameter );
        SiiMhlTxDeviceIsr();
        #if 0
        if( MHL_TX_EVENT_NONE != event )
        {
            AppRcpDemo( event, eventParameter);
        }
        #endif
        msleep(20);
        }
       return 0;
}


/*****************************************************************************/
/**
 * @brief Start driver's event monitoring thread.
 *
 *****************************************************************************/
void StartEventThread(void)
{
    gDriverContext.pTaskStruct = kthread_run(SiI9244_mhl_loop,
                                             &gDriverContext,
                                             MHL_DRIVER_NAME);
}


/*****************************************************************************/
/**
 * @brief Stop driver's event monitoring thread.
 *
 *****************************************************************************/
void  StopEventThread(void)
{
    kthread_stop(gDriverContext.pTaskStruct);

}
#endif

static struct i2c_device_id mhl_Sii9244_idtable[] = {
    {"mhl_Sii9244_page0", 0},
    {"mhl_Sii9244_page1", 0},
    {"mhl_Sii9244_page2", 0},
    {"mhl_Sii9244_cbus", 0},
};


/*
 * i2c client ftn.
 */
static int __devinit mhl_Sii9244_probe(struct i2c_client *client,
            const struct i2c_device_id *dev_id)
{
    int ret = 0;
    uint8_t     pollIntervalMs;

    int gpio_mhl_int = -1;
    TX_API_PRINT((KERN_INFO "%s:%d:\n", __func__,__LINE__));
    init_PAGE_values(); 

/*
    init_timer(&g_mhl_1ms_timer);
    g_mhl_1ms_timer.function = TimerTickHandler;
    g_mhl_1ms_timer.expires = jiffies + 10*HZ;
    add_timer(&g_mhl_1ms_timer);
*/
    if(match_id(&mhl_Sii9244_idtable[0], client))
    {
        mhl_Sii9244_page0 = client;
        dev_info(&client->adapter->dev, "attached %s "
            "into i2c adapter successfully\n", dev_id->name);
    }
    else if(match_id(&mhl_Sii9244_idtable[1], client))
    {
        mhl_Sii9244_page1 = client;
        dev_info(&client->adapter->dev, "attached %s "
            "into i2c adapter successfully \n", dev_id->name);
    }
    else if(match_id(&mhl_Sii9244_idtable[2], client))
    {
        mhl_Sii9244_page2 = client;
        dev_info(&client->adapter->dev, "attached %s "
            "into i2c adapter successfully \n", dev_id->name);
    }
    else if(match_id(&mhl_Sii9244_idtable[3], client))
    {
        mhl_Sii9244_cbus = client;
        dev_info(&client->adapter->dev, "attached %s "
            "into i2c adapter successfully\n", dev_id->name);

    }
    else
    {
        dev_info(&client->adapter->dev, "invalid i2c adapter: can not found dev_id matched\n");
        return -EIO;
    }


    if(mhl_Sii9244_page0 != NULL
        && mhl_Sii9244_page1 != NULL
        && mhl_Sii9244_page2 != NULL
        && mhl_Sii9244_cbus != NULL)
    {
        // Announce on RS232c port.
        //
        printk("\n============================================\n");
        printk("SiI9244 Linux Driver V1.22 \n");
        printk("============================================\n");

        //
        // Initialize the registers as required. Setup firmware vars.
        //


        if(false == Sii9244_mhl_reset())
            return -EIO;

        //HalTimerInit ( );
        //HalTimerSet (TIMER_POLLING, MONITORING_PERIOD);


        SiiMhlTxInitialize( pollIntervalMs = MONITORING_PERIOD);

        #ifdef SiI9244DRIVER_INTERRUPT_MODE
        gpio_mhl_int = get_gpio_num_by_name("GPIO_MHL_INT");
        if(gpio_mhl_int < 0)
        {
            printk(KERN_WARNING"%s:%d:get GPIO_MHL_INT failed\n",__func__,__LINE__);
            return -EIO;
        }
        mhl_Sii9244_page0->irq = OMAP_GPIO_IRQ(gpio_mhl_int);
        ret = request_threaded_irq(mhl_Sii9244_page0->irq, NULL, Sii9244_mhl_interrupt,IRQF_TRIGGER_LOW | IRQF_ONESHOT,//IRQ_TYPE_EDGE_FALLING,//IRQF_TRIGGER_LOW | IRQF_ONESHOT,//IRQ_TYPE_LEVEL_LOW,  //IRQ_TYPE_EDGE_FALLING,
                      mhl_Sii9244_page0->name, mhl_Sii9244_page0);
        if (ret){
            printk(KERN_INFO "%s:%d:Sii9244 interrupt failed\n", __func__,__LINE__);
            free_irq(mhl_Sii9244_page0->irq, mhl_Sii9244_page0->name);
            }

        else{
            enable_irq_wake(mhl_Sii9244_page0->irq);
            //disable_irq_nosync(mhl_Sii9244_page0->irq);
            printk(KERN_INFO "%s:%d:Sii9244 interrupt is sucessful\n", __func__,__LINE__);
            }

        #else
        StartEventThread();        /* begin monitoring for events */
        #endif

    }
	mhl_initialized = true;
    return ret;
}

static int mhl_Sii9244_remove(struct i2c_client *client)
{
    dev_info(&client->adapter->dev, "detached s5p_mhl "
        "from i2c adapter successfully\n");
    return 0;
}

static int mhl_Sii9244_suspend(struct i2c_client *cl, pm_message_t mesg)
{
    return 0;
};

static int mhl_Sii9244_resume(struct i2c_client *cl)
{
    return 0;
};


MODULE_DEVICE_TABLE(i2c, mhl_Sii9244_idtable);

static struct i2c_driver mhl_Sii9244_driver = {
    .driver = {
        .name = "Sii9244_Driver",
    },
    .id_table     = mhl_Sii9244_idtable,
    .probe         = mhl_Sii9244_probe,
    .remove     = __devexit_p(mhl_Sii9244_remove),

    .suspend    = mhl_Sii9244_suspend,
    .resume     = mhl_Sii9244_resume,
};

static int __init mhl_Sii9244_init(void)
{
    return i2c_add_driver(&mhl_Sii9244_driver);
}

static void __exit mhl_Sii9244_exit(void)
{
    i2c_del_driver(&mhl_Sii9244_driver);
}


late_initcall(mhl_Sii9244_init);
module_exit(mhl_Sii9244_exit);

MODULE_VERSION("1.22");
MODULE_AUTHOR("gary <qiang.yuan@siliconimage.com>, Silicon image SZ office, Inc.");
MODULE_DESCRIPTION("sii9244 transmitter Linux driver");
MODULE_ALIAS("platform:MHL_sii9244");
