#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "time.h"
#include "sys/time.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "ad7606.h"

esp_err_t ret;
spi_device_handle_t spi;

QueueHandle_t XQueue_SPI;


//define from SPP
#define SPP_TAG "SPP_INITIATOR_DEMO"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_INITIATOR"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA    /*Choose show mode: show data or speed*/

//define from SPI
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_RST  33
#define PIN_NUM_CVAB 2
#define PIN_NUM_BUSY 16 
#define PIN_NUM_RAGE 32
#define PIN_NUM_OS0  25
#define PIN_NUM_OS1  26
#define PIN_NUM_OS2  27
#define CH_NUM       8u  //u表示无符号整形
#define DMA_CHAN     2
#define AD_HOST    SPI3_HOST

//val from SPP
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB; /*!< When data is coming, a callback will come with data */
static struct timeval time_new, time_old;//timeval结构体中含s和us
static long data_num = 0;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;//安全连接，认证要求，这里结合了加密连接的功能，我没开
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;//主机角色，发送数据
esp_bd_addr_t peer_bd_addr = {0};//蓝牙设备地址存放数组
static uint8_t peer_bdname_len;//设备名长度（对端）
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];//[249]
static const char remote_device_name[] = "ESP_SPP_ACCEPTOR";//连接的设备名
static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;//一般查询方式
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
#define SPP_DATA_LEN 16//发送数据的长度;一次不超20（00-13）；十六进制
#else
#define SPP_DATA_LEN ESP_SPP_MAX_MTU
#endif
static uint8_t spp_data[SPP_DATA_LEN];//存放发送数据的数组

//val from SPI
uint8_t ADC_val[CH_NUM * 2];

void AD7606_IOset()
{
    gpio_set_direction(PIN_NUM_RST,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_CVAB,GPIO_MODE_OUTPUT);
    
    

    gpio_set_direction(PIN_NUM_CS,GPIO_MODE_OUTPUT);

    gpio_set_direction(PIN_NUM_BUSY,GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_NUM_BUSY);
    gpio_pulldown_dis(PIN_NUM_BUSY);

    gpio_set_direction(PIN_NUM_RAGE,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_OS0,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_OS1,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_OS2,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS,1);
    gpio_set_level(PIN_NUM_CVAB,1);
    

    gpio_set_level(PIN_NUM_RST,0);
}


void AD7606_Init()
{
    AD7606_IOset();
    AD7606_SetInputRange(0);
    // AD7606_SetInputRange(1);//量程（-10~+10）

    AD7606_OSset();
    AD7606_Reset();
}

//设置输入范围；0表示±5V, 1表示±10V.
void AD7606_SetInputRange(int range)
{
    gpio_set_level(PIN_NUM_RAGE,range);
}

/*过采样设置：
 *OS[2:0]:
 *000表示无过采样，最大200Ksps采样速率:数据正常
 *001表示2倍过采样， 也就是硬件内部采集2个样本求平均:数据正常
 *010表示4倍过采样， 也就是硬件内部采集4个样本求平均:数据不正常
 *011表示8倍过采样， 也就是硬件内部采集8个样本求平均:数据不正常
 *100表示16倍过采样， 也就是硬件内部采集16个样本求平均:数据不正常
 *101表示32倍过采样， 也就是硬件内部采集32个样本求平均:数据不正常
 *110表示64倍过采样， 也就是硬件内部采集64个样本求平均:数据不正常
 */
void AD7606_OSset()
{
    gpio_set_level(PIN_NUM_OS0,0);
    gpio_set_level(PIN_NUM_OS1,0);
    gpio_set_level(PIN_NUM_OS2,0);
}

void AD7606_Reset()
{
    gpio_set_level(PIN_NUM_CS,1);
    gpio_set_level(PIN_NUM_CVAB,1);
    


    gpio_set_level(PIN_NUM_RST,0);
    gpio_set_level(PIN_NUM_RST,1);
    gpio_set_level(PIN_NUM_RST,0);//reset的时间不应少于50ns
}

void AD7606_StartConv()
{
    gpio_set_level(PIN_NUM_CVAB,0);
    gpio_set_level(PIN_NUM_CVAB,0);
    gpio_set_level(PIN_NUM_CVAB,0);
    gpio_set_level(PIN_NUM_CVAB,1);
}

esp_err_t spi_read(spi_device_handle_t spi, uint8_t *data)
{
    spi_transaction_t t;

    gpio_set_level(PIN_NUM_CS, 0);

    memset(&t, 0, sizeof(t));
    t.length=8;
    // t.length=8;//关键，一次读一字节；
    t.rxlength=8;
    t.flags = SPI_TRANS_USE_RXDATA;
    // t.user = (void*)1;
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );
    
    *data = t.rx_data[0];
    
    
    gpio_set_level(PIN_NUM_CS, 1);

    return ret;
}

static void print_speed(void)//计算传输速率并打印，数据模式应该不用
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;//time_old_s  秒+微秒
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;//time_new_s  秒+微秒
    float time_interval = time_new_s - time_old_s;                    //事件间隔（打印的？）
    float speed = data_num * 8 / time_interval / 1000.0;              //数据量*8/时间间隔/1000=传输速率（kbits/s）
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)//eir data:设备身份寄存器
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)//GAP回调函数

{
    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
        esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        for (int i = 0; i < param->disc_res.num_prop; i++){
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                esp_log_buffer_char(SPP_TAG, peer_bdname, peer_bdname_len);
                if (strlen(remote_device_name) == peer_bdname_len
                    && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0) {
                    memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                    esp_spp_start_discovery(peer_bd_addr);
                    esp_bt_gap_cancel_discovery();
                }
            }
        }
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        ESP_LOGW(SPP_TAG, "To confirm the value, type `spp ok;`");
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        ESP_LOGW(SPP_TAG, "Waiting responce...");
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        ESP_LOGW(SPP_TAG, "To input the key, type `spp key xxxxxx;`");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default:
        break;
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)//时间判断和处理(回调函数)
{
    switch (event) {
    case ESP_SPP_INIT_EVT://设置名称
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);//设置连接模式：可连接；发现模式：一般可发现；
        esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);//查询模式，查询长度；回复长度

        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
        }
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        esp_spp_write(param->open.handle, SPP_DATA_LEN, spp_data);
        gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");
        break;
    case ESP_SPP_CONG_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong=%d", param->cong.cong);
#endif
        if (param->cong.cong == 0) {
            esp_spp_write(param->cong.handle, SPP_DATA_LEN, spp_data);
        }//写数据进缓冲区
        break;
    case ESP_SPP_WRITE_EVT://写入
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len , param->write.cong);//bool cong;TRUE,阻塞. FALSE,不阻塞
        esp_log_buffer_hex("",spp_data,SPP_DATA_LEN);//十六进制打印缓冲区内容（tag，指向缓冲区数组的指针，数组长度）
        //esp_log_buffer_char("",spp_data,SPP_DATA_LEN);
       
        
#else
        gettimeofday(&time_new, NULL);//检索当前时间，精度微秒
        data_num += param->write.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }
#endif
        vTaskDelay(100);//添加的延时

        // for (int i = 0; i < SPP_DATA_LEN; ++i) {
        //     spp_data[i] = rand();
        // }//发送的数组
  
        // xQueueReceive( XQueue_SPI, spp_data, ( TickType_t ) 10 );//传几个就传不过来了，应设置portMAX_DELAY一直到有数据来
        xQueueReceive( XQueue_SPI, spp_data, portMAX_DELAY );

   

        if (param->write.cong == 0) {
            esp_spp_write(param->write.handle, SPP_DATA_LEN, spp_data);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void TaskSPI(void *pvParameters)
{
    

    AD7606_Init();//ad初始化
    printf("+----------+----------+----------+----------+----------+----------+----------+----------+\n");
    printf("|    V1    |    V2    |    V3    |    V4    |    V5    |    V6    |    V7    |    V8    |\n");
    printf("+----------+----------+----------+----------+----------+----------+----------+----------+\n");
    
   
    while (1)
    {      
        if( gpio_get_level(PIN_NUM_BUSY)==0 )
            AD7606_StartConv();
        // if(gpio_get_level(PIN_NUM_BUSY!=1)) //刚开始的笔误，实际上就是if(1)
        // if(gpio_get_level(PIN_NUM_BUSY)==0)
        
            // ret = spi_read(spi, &ADC_val);
            for (int i = 0; i < CH_NUM*2; i++) 
            {
                
                ret = spi_read(spi, &ADC_val[i]);
                ESP_ERROR_CHECK(ret);

                uint16_t tmp;
				float disp;
				tmp = ADC_val[2 * i + 1] + (ADC_val[2 * i] <<8);
                // tmp = ADC_val[i];
				disp = 1.0 * tmp * 10 / 65535; 
				// disp = 1.0 * tmp * 20 / 65535; 

                if(i<8){
                /*量程为-10~+10*/    
                // if(disp >= 10){
				// 	disp = 20 - disp;

                /*量程为-5~+5*/
                if(disp >= 5){
					disp = 10 - disp;

					printf ("|-%f ", disp);
				} 
				else{
					printf ("| %f ", disp);
                }

                }
            }

            printf("|\n");
            printf ("+----------+----------+----------+----------+----------+----------+----------+----------+\n") ;
            
            xQueueSend( XQueue_SPI, ADC_val, 0 );

            // ESP_LOGI(TAG, "Read: %s", ADC_val);
            vTaskDelay(100);//改到1也没问题
            
        
    }    
    
}

void app_main(void)
{
    // esp_err_t ret;
    // spi_device_handle_t spi;

    XQueue_SPI = xQueueCreate(3,16);
    xTaskCreate(TaskSPI, "SPIdata", 4096, NULL, 1, NULL);

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,                // MISO信号线
        .mosi_io_num = -1,                          // MOSI信号线
        .sclk_io_num = PIN_NUM_CLK,                 // SCLK信号线
        .quadwp_io_num = -1,                        // WP信号线，专用于QSPI的D2
        .quadhd_io_num = -1,                        // HD信号线，专用于QSPI的D3
        .max_transfer_sz = 64*8,                    // 最大传输数据大小
    };

    //spi_device_interface_config_t用于配置SPI协议情况
    //需要根据从设备的数据手册进行设置
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = SPI_MASTER_FREQ_10M,      // Clock out at 10 MHz,
        .mode = 2,                                  // SPI mode 1（CPOL = 1, CPHA = 0）
        .spics_io_num = -1,
        .queue_size = 7,                            // 传输队列大小，决定了等待传输数据的数量
    };

    //初始化SPI总线
    ret = spi_bus_initialize(AD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    
    //添加从设备
    ret = spi_bus_add_device(AD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    


        
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IN;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    if (iocap == ESP_BT_IO_CAP_IN || iocap == ESP_BT_IO_CAP_IO) {
        console_uart_init();
    }
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    



        //这样写可以传一次过去但不会更新状态

        // if( gpio_get_level(PIN_NUM_BUSY)==0 ){
        //     AD7606_StartConv();
        //     // if(1) 
        //     // ret = spi_read(spi, &ADC_val);
        //     for (int i = 0; i < CH_NUM*2; i++) 
        //     {
                
        //         ret = spi_read(spi, &ADC_val[i]);
        //         ESP_ERROR_CHECK(ret);

        //         uint16_t tmp;
		// 		float disp;
		// 		tmp = ADC_val[2 * i + 1] + (ADC_val[2 * i] <<8);
        //         // tmp = ADC_val[i];
		// 		disp = 1.0 * tmp * 10 / 65535; 
		// 		// disp = 1.0 * tmp * 20 / 65535; 

        //         if(i<8){
        //         /*量程为-10~+10*/    
        //         // if(disp >= 10){
		// 		// 	disp = 20 - disp;

        //         /*量程为-5~+5*/
        //         if(disp >= 5){
		// 			disp = 10 - disp;

		// 			printf ("|-%f ", disp);
		// 		} 
		// 		else{
		// 			printf ("| %f ", disp);
        //         }

        //         }
            

        //     printf("|\n");
        //     printf ("+----------+----------+----------+----------+----------+----------+----------+----------+\n") ;

        //     // ESP_LOGI(TAG, "Read: %s", ADC_val);
        //     vTaskDelay(100);//改到1也没问题
        //     }
        
        //     }    

}
