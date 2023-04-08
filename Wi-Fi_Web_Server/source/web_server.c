/*******************************************************************************
* File Name: web_server.c
*
* Description: This file contains the necessary functions to configure the device 
*              in SoftAP mode and starts an HTTP server. The device can be 
*              provisioned to connect to an AP, after performing a scan for 
*              available APs, by using the credentials entered via HTTP client. 
*              Once the device is connected to AP, starts an HTTP server which 
*              processes GET and POST request from the HTTP client. 
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>

/* Secure Sockets header file */
#include "cy_secure_sockets.h"
#include "cy_tls.h"

/* Wi-Fi connection manager header files */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* HTTP server task header file. */
#include "web_server.h"
#include "cy_http_server.h"
#include "cy_log.h"

/* Standard C header file */
#include <string.h>
#include <ctype.h>

/* HTTP server task header file. */
#include "cy_http_server.h"
#include "html_web_page.h"
#include "web_server.h"


#define PUMP1_ON_TIME (1000u)
#define PUMP2_ON_TIME (1500u)
#define PUMP3_ON_TIME (2000u)
#define PUMP4_ON_TIME (3000u)

#define PUMP_1_PIN   (P0_2)
#define PUMP_2_PIN   (P0_3)
#define PUMP_3_PIN   (P13_4)
#define PUMP_4_PIN   (P13_5)

#define VPLUS_CHANNEL_0  (P10_0)
#define VPLUS_CHANNEL_1  (P10_1)
#define VPLUS_CHANNEL_2  (P10_2)
#define VPLUS_CHANNEL_3  (P10_3)

/*******************************************************************************
* Global Variables
********************************************************************************/

#ifdef ENABLE_TFT

/* Global variable to hold display row position. */
uint16_t row = TOP_DISPLAY;

/* Row value to print sensor values TFT display. */
uint16_t sensor_row_print = 0;

/* Light sensor object. */
//extern mtb_light_sensor_t light_sensor_obj;

/* Pin mapping used in TFT display */
const mtb_st7789v_pins_t tft_pins =
{
    .db08 = CY8CKIT_028_TFT_PIN_DISPLAY_DB8,
    .db09 = CY8CKIT_028_TFT_PIN_DISPLAY_DB9,
    .db10 = CY8CKIT_028_TFT_PIN_DISPLAY_DB10,
    .db11 = CY8CKIT_028_TFT_PIN_DISPLAY_DB11,
    .db12 = CY8CKIT_028_TFT_PIN_DISPLAY_DB12,
    .db13 = CY8CKIT_028_TFT_PIN_DISPLAY_DB13,
    .db14 = CY8CKIT_028_TFT_PIN_DISPLAY_DB14,
    .db15 = CY8CKIT_028_TFT_PIN_DISPLAY_DB15,
    .nrd  = CY8CKIT_028_TFT_PIN_DISPLAY_NRD,
    .nwr  = CY8CKIT_028_TFT_PIN_DISPLAY_NWR,
    .dc   = CY8CKIT_028_TFT_PIN_DISPLAY_DC,
    .rst  = CY8CKIT_028_TFT_PIN_DISPLAY_RST
};
#endif

/* Holds the IP address and port number details of the socket for the HTTP server. */
cy_socket_sockaddr_t http_server_ip_address;

/* Pointer to HTTP event stream used to send device data to client. */
cy_http_response_stream_t* http_event_stream;

/* Wi-Fi network interface. */
cy_network_interface_t nw_interface;

/* HTTP server instance. */
cy_http_server_t http_ap_server;

/* HTTP server instance. */
cy_http_server_t http_sta_server;

/*Buffer to store SSID*/
uint8_t wifi_ssid[WIFI_SSID_LEN] = {0};

/*Buffer to store Password*/
uint8_t wifi_pwd[WIFI_PWD_LEN] = {0}; 

/*Buffer to store HTTP data*/
char buffer[BUFFER_LENGTH] = {0};

/* Holds the response handler for HTTP GET and POST request from the client 
* to implement Wi-Fi scan and Wi-Fi connect funtionality. 
*/
cy_resource_dynamic_data_t http_wifi_resource;

/* Flag to indicate if scan has completed.*/
volatile bool scan_complete_flag = false;

/* Flag to indicate if device has been configured. */
volatile bool device_configured = true;

/* Buffer to store ssid  */
static char ssid_buff[BUFFER_LENGTH];

/*Variable to indicate re-configuration request*/
volatile int8_t reconfiguration_request = 0;

/* Flag to indicate status of increase pwm value command. */
volatile bool increase_pwm = false;

/* Flag to indicate status of decrease pwm value command. */
volatile bool decrease_pwm = false;

/* Array to store Wi-Fi connect response. */
static char http_wifi_connect_response[WIFI_CONNECT_RESPONSE_LENGTH] = {0};

/* Array to store Wi-Fi scan response. */
static char http_scan_response[MAX_WIFI_SCAN_HTTP_RESPONSE_LENGTH] = {0};


typedef enum
{
    NO_PUMP,
	PUMP_1,
    PUMP_2,
    PUMP_3,
    PUMP_4
} pump_num_t;


volatile pump_num_t pump_action = NO_PUMP;


/*******************************************************************************
 * Function Name: process_sse_handler
 *******************************************************************************
 * Summary:
 *  Handler for enabling server sent events
 *
 * Parameters:
 *  url_path - Pointer to the HTTP URL path.
 *  url_parameters - Pointer to the HTTP URL query string.
 *  stream - Pointer to the HTTP response stream.
 *  arg - Pointer to the argument passed during HTTP resource registration.
 *  http_message_body - Pointer to the HTTP data from the client.
 *
 * Return:
 *  int32_t - Returns HTTP_REQUEST_HANDLE_SUCCESS if the request from the client
 *  was handled successfully. Otherwise, it returns HTTP_REQUEST_HANDLE_ERROR.
 *
 *******************************************************************************/
int32_t process_sse_handler( const char* url_path, const char* url_parameters,
                                   cy_http_response_stream_t* stream, void* arg,
                                   cy_http_message_body_t* http_message_body )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Assign the incoming stream to http_event_stream pointer */
    http_event_stream = stream;

    /* Enable chunked transfer encoding on the HTTP stream */
    result = cy_http_server_response_stream_enable_chunked_transfer( http_event_stream );
    PRINT_AND_ASSERT(result, "HTTP server event failed to enable chunked transfer\r\n");

    result = cy_http_server_response_stream_write_header( http_event_stream, CY_HTTP_200_TYPE,
                                                CHUNKED_CONTENT_LENGTH, CY_HTTP_CACHE_DISABLED,
                                                MIME_TYPE_TEXT_EVENT_STREAM );
    PRINT_AND_ASSERT(result, "HTTP server event failed to write stream header\r\n");

    return result;
}

/*******************************************************************************
 * Function Name: softap_resource_handler
 *******************************************************************************
 * Summary:
 *  Handles HTTP GET, POST, and PUT requests from the client.
 *  HTTP GET sends the HTTP startup webpage as a response to the client.
 *  HTTP POST extracts the credentials from the HTTP data from the client 
 *  and tries to connect to the AP.
 *  HTTP PUT sends an error message as a response to the client if the resource
 *  registration is unsuccessful.
 *
 * Parameters:
 *  url_path - Pointer to the HTTP URL path.
 *  url_parameters - Pointer to the HTTP URL query string.
 *  stream - Pointer to the HTTP response stream.
 *  arg - Pointer to the argument passed during HTTP resource registration.
 *  http_message_body - Pointer to the HTTP data from the client.
 *
 * Return:
 *  int32_t - Returns HTTP_REQUEST_HANDLE_SUCCESS if the request from the client
 *  was handled successfully. Otherwise, it returns HTTP_REQUEST_HANDLE_ERROR.
 *
 *******************************************************************************/
int32_t softap_resource_handler(const char *url_path,
                                 const char *url_parameters,
                                 cy_http_response_stream_t *stream,
                                 void *arg,
                                 cy_http_message_body_t *http_message_body)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int32_t status = HTTP_REQUEST_HANDLE_SUCCESS;

    switch (http_message_body->request_type)
    {
    case CY_HTTP_REQUEST_GET:

        /* If device is not configured send the initial page */
        if(!device_configured)
        {
            /* The start up page of the HTTP client will be sent as an initial response
             * to the GET request.
             */
            //result = cy_http_server_response_stream_write_payload(stream, HTTP_SOFTAP_STARTUP_WEBPAGE, sizeof(HTTP_SOFTAP_STARTUP_WEBPAGE) - 1);
            if (CY_RSLT_SUCCESS != result)
            {
                ERR_INFO(("Failed to send the HTTP GET response.\r\n"));
            }
        }
        else
        {
            /* Send the data of the device */
            result = cy_http_server_response_stream_write_payload(stream, SOFTAP_DEVICE_DATA, sizeof(SOFTAP_DEVICE_DATA) - 1);
            if (CY_RSLT_SUCCESS != result)
            {
                ERR_INFO(("Failed to send the HTTP GET response.\n"));
            }
        }
        break;

    case CY_HTTP_REQUEST_POST:

        if(!device_configured)
        {
            /* The device tries to connect to the AP using the credentials sent via HTTP
             * webpage.
             */
            //result = wifi_extract_credentials(http_message_body->data, http_message_body->data_length,stream);
            //result = wifi_extract_credentials();
        }
        else
        {
            /* Compare the input from client to turn on a pump. */
            if(!strncmp((char *)http_message_body->data, PUMP1, 5))
			{
            	APP_INFO(("Pump1 command received\n"));
            	pump_action = PUMP_1;
			}
            else if(!strncmp((char *)http_message_body->data, PUMP2, 5))
			{
            	APP_INFO(("Pump2 command received\n"));
            	pump_action = PUMP_2;
			}
            else if(!strncmp((char *)http_message_body->data, PUMP3, 5))
			{
            	APP_INFO(("Pump3 command received\n"));
            	pump_action = PUMP_3;
			}
            else if(!strncmp((char *)http_message_body->data, PUMP4, 5))
			{
            	APP_INFO(("Pump4 command received\n"));
            	pump_action = PUMP_4;
			}


            /* Send the HTTP response. */
            result = cy_http_server_response_stream_write_payload(stream, HTTP_HEADER_204, sizeof(HTTP_HEADER_204) - 1);
            if (CY_RSLT_SUCCESS != result)
            {
                ERR_INFO(("Failed to send the HTTP POST response.\n"));
            }
        }
    break;

    default:
        ERR_INFO(("Received invalid HTTP request method. Supported HTTP methods are GET, POST, and PUT.\n"));
        
        break;

    }

    if (CY_RSLT_SUCCESS != result)
    {
        status = HTTP_REQUEST_HANDLE_ERROR;
    }

    return status;
}

/*******************************************************************************
 * Function Name: wifi_resource_handler
 *******************************************************************************
 * Summary:
 *  Handles HTTP GET, POST, and PUT requests from the client.
 *  HTTP GET performs scan for available networks(APs) and sends the list of 
 *  available networks as a response to the client.
 *  HTTP POST extracts the credentials from the HTTP data from the client 
 *  and tries to connect to the AP.
 *
 * Parameters:
 *  url_path - Pointer to the HTTP URL path.
 *  url_parameters - Pointer to the HTTP URL query string.
 *  stream - Pointer to the HTTP response stream.
 *  arg - Pointer to the argument passed during HTTP resource registration.
 *  http_message_body - Pointer to the HTTP data from the client.
 *
 * Return:
 *  int32_t - Returns HTTP_REQUEST_HANDLE_SUCCESS if the request from the client
 *  was handled successfully. Otherwise, it returns HTTP_REQUEST_HANDLE_ERROR.
 *
 *******************************************************************************/
static int32_t wifi_resource_handler(const char *url_path,
                                     const char *url_parameters,
                                     cy_http_response_stream_t *stream,
                                     void *arg,
                                     cy_http_message_body_t *http_message_body)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int32_t status = HTTP_REQUEST_HANDLE_SUCCESS;
    
    switch (http_message_body->request_type)
    {
    case CY_HTTP_REQUEST_GET:

        /* Scan for the available networks in response to the HTTP GET request. */
        scan_for_available_aps(stream);
        
        break;

    case CY_HTTP_REQUEST_POST:

        /* The device tries to connect to the AP using the credentials sent via HTTP
         * webpage.
         */
//result = cy_http_server_response_stream_write_payload(stream, HTTP_DEVICE_DATA_REDIRECT_WEBPAGE, sizeof(HTTP_DEVICE_DATA_REDIRECT_WEBPAGE));
        if (CY_RSLT_SUCCESS != result)
        {
            ERR_INFO(("Failed to send the HTTP POST response.\n"));
        }

        /* Set device configured flag to true */
        device_configured = true;
        reconfiguration_request = SERVER_RECONFIGURE_REQUESTED;
        cy_wcm_stop_ap();
        break;

    default:
        ERR_INFO(("Wi-Fi Scan: Received invalid HTTP request method. Supported HTTP methods are GET, POST, and PUT.\n"));
        
        break;

    }

    if (CY_RSLT_SUCCESS != result)
    {
        status = HTTP_REQUEST_HANDLE_ERROR;
    }

    return status;
}

/********************************************************************************
 * Function Name: wifi_extract_credentials
 ********************************************************************************
 * Summary:
 *  The function extracts the credentials entered via HTTP webpage. Switches to STA 
 *  mode then connects to the same credentials.
 *
 * Parameters:
 *  const uint8_t* data : The HTTP data that contains ssid and password that is
 *  entered from the HTTP webpage.
*   uint32_t data_len : The length of the HTTP response.
 *
 * Return:
 *  void
 *
 *******************************************************************************/
//cy_rslt_t wifi_extract_credentials(const uint8_t *data, uint32_t data_len, cy_http_response_stream_t *stream)
cy_rslt_t wifi_extract_credentials(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = start_sta_mode();

    return result;
}

/*******************************************************************************
 * Function Name: start_sta_mode
 *******************************************************************************
 * Summary:
 *  The function attempts to connect to Wi-Fi until a connection is made or
 *  MAX_WIFI_RETRY_COUNT attempts have been made.
 * 
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the HTTP server is configured
 *  successfully, otherwise, it returns CY_RSLT_TYPE_ERROR.
 *
 *******************************************************************************/
cy_rslt_t start_sta_mode()
{
    cy_rslt_t result;
    cy_wcm_connect_params_t connect_param;
    cy_wcm_ip_address_t ip_address;
    bool wifi_conct_stat = false;

    /*Disconnect from the currently connected AP if any*/
    wifi_conct_stat = cy_wcm_is_connected_to_ap();
    if(wifi_conct_stat)
    {
        cy_wcm_disconnect_ap();
    }

	const char sreejith_ssid[] = "Byte Me";
 	const char sreejith_pwd[] = "Indinkanwetrust!";
	strcpy((char*)wifi_ssid,sreejith_ssid);
	strcpy((char*)wifi_pwd,sreejith_pwd);

    memset(&connect_param, 0, sizeof(cy_wcm_connect_params_t));
    memset(&ip_address, 0, sizeof(cy_wcm_ip_address_t));
   
    memcpy(connect_param.ap_credentials.SSID, wifi_ssid, sizeof(wifi_ssid));
    memcpy(connect_param.ap_credentials.password, wifi_pwd, sizeof(wifi_pwd));
    connect_param.ap_credentials.security = CY_WCM_SECURITY_WPA2_AES_PSK;

    /* Attempt to connect to Wi-Fi until a connection is made or
     * MAX_WIFI_RETRY_COUNT attempts have been made.
     */
    for (uint32_t conn_retries = 0; conn_retries < MAX_WIFI_RETRY_COUNT; conn_retries++)
    {
        result = cy_wcm_connect_ap(&connect_param, &ip_address);
        if (result == CY_RSLT_SUCCESS)
        {
            APP_INFO(("Successfully connected to Wi-Fi network '%s'.\n", connect_param.ap_credentials.SSID));
            break;
        }
        ERR_INFO(("Connection to Wi-Fi network failed with error code %d. Retrying in %d ms...\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC));

        vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
    }

    return result;
}

/*******************************************************************************
 * Function Name: reconfigure_http_server
 *******************************************************************************
 * Summary:
 * The function deletes the existing HTTP server instance (http_ap_server), and 
 * starts a new HTTP server instance (http_sta_server). After registering 
 * dynamic URL handler (process_sse_handler and http_get_post_resource) to handle 
 * the HTTP GET, POST, and PUT requests.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the HTTP server is configured
 *  successfully, otherwise, it returns CY_RSLT_TYPE_ERROR.
 *
 *******************************************************************************/
cy_rslt_t reconfigure_http_server(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_wcm_ip_address_t ip_addr;

    /* Holds the response handler for HTTP GET and POST request from the client. */
    cy_resource_dynamic_data_t http_get_post_resource;

    /* Holds the response handler for dynamic SSE resource. */
    cy_resource_dynamic_data_t dynamic_sse_resource;

    /* IP address of SoftAp. */
    result = cy_wcm_get_ip_addr(CY_WCM_INTERFACE_TYPE_STA, &ip_addr);
    PRINT_AND_ASSERT(result, "cy_wcm_get_ip_addr failed for creating HTTP server...! \n");

    http_server_ip_address.ip_address.ip.v4 = ip_addr.ip.v4;
    http_server_ip_address.ip_address.version = CY_SOCKET_IP_VER_V4;

    /* Add IP address information to network interface object. */
    nw_interface.object = (void *)&http_server_ip_address;
    nw_interface.type = CY_NW_INF_TYPE_WIFI;

    /* Initialize secure socket library. */
    result = cy_http_server_network_init();

    /* Allocate memory needed for secure HTTP server. */
    result = cy_http_server_create(&nw_interface, HTTP_PORT, MAX_SOCKETS, NULL, &http_sta_server);
    PRINT_AND_ASSERT(result, "Failed to allocate memory for the HTTP server.\n");

    /* Configure server sent events*/
    dynamic_sse_resource.resource_handler = process_sse_handler;
    dynamic_sse_resource.arg = NULL;
    result = cy_http_server_register_resource( http_sta_server,
                                                (uint8_t*) "/events",
                                                (uint8_t*)"text/event-stream",
                                                CY_RAW_DYNAMIC_URL_CONTENT,
                                                &dynamic_sse_resource);
    PRINT_AND_ASSERT(result, "Failed to register a resource.\n");
    
    /* Configure dynamic resource handler. */
    http_get_post_resource.resource_handler = softap_resource_handler;
    http_get_post_resource.arg = NULL;

    /* Register all the resources with the secure HTTP server. */
    result = cy_http_server_register_resource(http_sta_server,
                                              (uint8_t *)"/",
                                              (uint8_t *)"text/html",
                                              CY_DYNAMIC_URL_CONTENT,
                                              &http_get_post_resource);
    PRINT_AND_ASSERT(result, "Failed to register a resource.\n");

    /* Start the HTTP server. */
    result = cy_http_server_start(http_sta_server);
    PRINT_AND_ASSERT(result, "Failed to start the HTTP server.\n");
   
    return result;
}

/*******************************************************************************
* Macros
*******************************************************************************/

/* Number of scans every time ADC read is initiated */
#define NUM_SCAN                         (1)

/* Conversion factor */
#define MICRO_TO_MILLI_CONV_RATIO        (1000u)

/* Acquistion time in nanosecond */
#define ACQUISITION_TIME_NS              (1000u)

/* ADC Scan delay in millisecond */
#define ADC_SCAN_DELAY_MS                (500u)

/*******************************************************************************
*       Enumerated Types
*******************************************************************************/
/* ADC Channel constants*/
enum ADC_CHANNELS
{
  CHANNEL_0 = 0,
  CHANNEL_1,
  CHANNEL_2,
  CHANNEL_3,
  NUM_CHANNELS
} adc_channel;

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* ADC Object */
cyhal_adc_t adc_obj;

/* ADC Channel 0 Object */
cyhal_adc_channel_t adc_chan_0_obj;
/* ADC Channel 1 Object */
cyhal_adc_channel_t adc_chan_1_obj;
/* ADC Channel 2 Object */
cyhal_adc_channel_t adc_chan_2_obj;
/* ADC Channel 3 Object */
cyhal_adc_channel_t adc_chan_3_obj;

/* Default ADC configuration */
const cyhal_adc_config_t adc_config = {
        .continuous_scanning=false, // Continuous Scanning is disabled
        .average_count=1,           // Average count disabled
        .vref=CYHAL_ADC_REF_VDDA,   // VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  // VNEG for Single ended channel set to VSSA
        .resolution = 12u,          // 12-bit resolution
        .ext_vref = NC,             // No connection
        .bypass_pin = NC };       // No connection


/* Asynchronous read complete flag, used in Event Handler */
static bool async_read_complete = false;

/* Variable to store results from multiple channels during asynchronous read*/
int32_t result_arr[NUM_CHANNELS * NUM_SCAN] = {0};

/* Variable to store ADC conversion result from channel 0 */
int32_t adc_result[4] = {0};

/*******************************************************************************
 * Function Name: adc_event_handler
 *******************************************************************************
 *
 * Summary:
 *  ADC event handler. This function handles the asynchronous read complete event
 *  and sets the async_read_complete flag to true.
 *
 * Parameters:
 *  void *arg : pointer to result list
 *  cyhal_adc_event_t event : ADC event type
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void adc_event_handler(void* arg, cyhal_adc_event_t event)
{
    if(0u != (event & CYHAL_ADC_ASYNC_READ_COMPLETE))
    {
        /* Set async read complete flag to true */
        async_read_complete = true;
    }
}
/*******************************************************************************
 * Function Name: adc_multi_channel_init
 *******************************************************************************
 *
 * Summary:
 *  ADC Multichannel initilization. This function initializes and configures
 *  channel 0 and channel 1 of ADC.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void adc_multi_channel_init(void)
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Initialize ADC. The ADC block which can connect to the channel 0 input pin is selected */
    result = cyhal_adc_init(&adc_obj, VPLUS_CHANNEL_0, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* ADC channel configuration */
    const cyhal_adc_channel_config_t channel_config = {
            .enable_averaging = false,  // Disable averaging for channel
            .min_acquisition_ns = ACQUISITION_TIME_NS, // Minimum acquisition time set to 1us
            .enabled = true };          // Sample this channel when ADC performs a scan

    /* Initialize a channel 0 and configure it to scan the channel 0 input pin in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, VPLUS_CHANNEL_0,
                                          CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize a channel 1 and configure it to scan the channel 1 input pin in single ended mode. */
    result = cyhal_adc_channel_init_diff(&adc_chan_1_obj, &adc_obj, VPLUS_CHANNEL_1,
            							  CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

	 /* Initialize a channel 2 and configure it to scan the channel 2 input pin in single ended mode. */
	 result = cyhal_adc_channel_init_diff(&adc_chan_2_obj, &adc_obj, VPLUS_CHANNEL_2,
										  CYHAL_ADC_VNEG, &channel_config);
	 if(result != CY_RSLT_SUCCESS)
	 {
		 CY_ASSERT(0);
	 }


	 /* Initialize a channel 3 and configure it to scan the channel 3 input pin in single ended mode. */
	 result = cyhal_adc_channel_init_diff(&adc_chan_3_obj, &adc_obj, VPLUS_CHANNEL_3,
										  CYHAL_ADC_VNEG, &channel_config);
	 if(result != CY_RSLT_SUCCESS)
	 {
		 CY_ASSERT(0);
	 }

	 /* Register a callback to handle asynchronous read completion */
	  cyhal_adc_register_callback(&adc_obj, &adc_event_handler, result_arr);

	  /* Subscribe to the async read complete event to process the results */
	  cyhal_adc_enable_event(&adc_obj, CYHAL_ADC_ASYNC_READ_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
}



/*******************************************************************************
* Function Name: server_task
********************************************************************************
* Summary:
*  Task that initializes the device as SoftAp, and starts the HTTP server
*
* Parameters:
*  arg - Unused.
*
* Return:
*  None.
*
*******************************************************************************/
void server_task(void *arg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    (void)arg;


    /* Initialize pump GPIOs */
    cyhal_gpio_init(PUMP_1_PIN,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,0u);
    cyhal_gpio_init(PUMP_2_PIN,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,0u);
    cyhal_gpio_init(PUMP_3_PIN,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,0u);
    cyhal_gpio_init(PUMP_4_PIN,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,0u);

    /* Initialize ADC channels */
    adc_multi_channel_init();

	/* Update ADC configuration */
	result = cyhal_adc_configure(&adc_obj, &adc_config);
	if(result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

#ifdef ENABLE_TFT
    result = mtb_st7789v_init8(&tft_pins);

    CY_ASSERT(result == CY_RSLT_SUCCESS);
#endif /* #ifdef ENABLE_TFT */

    char sensor_value_buffer[SENSOR_BUFFER_LENGTH];
    char http_response[MAX_HTTP_RESPONSE_LENGTH] = {0};

#ifdef ENABLE_TFT
    /*Initialize and setup TFT display */
    initialize_display();
#endif /* #ifdef ENABLE_TFT */

    /* Initialize the Wi-Fi device as a STA.*/
    cy_wcm_config_t config = {.interface = CY_WCM_INTERFACE_TYPE_AP_STA};
   
    /* Initialize the Wi-Fi device, Wi-Fi transport, and lwIP network stack.*/
    result = cy_wcm_init(&config);
    PRINT_AND_ASSERT(result,"cy_wcm_init failed...!\n");


    wifi_extract_credentials();
    reconfiguration_request = SERVER_RECONFIGURE_REQUESTED;
    
    /* Waits for queue message to register a new HTTP page resource.*/
    while (true)
    {
        if(SERVER_RECONFIGURED == reconfiguration_request)
        {

            if(decrease_pwm)
            {
                decrease_pwm = false;

            }
            if(increase_pwm)
            {
                increase_pwm = false;
            }

            if(pump_action == PUMP_1)
			   {
				   cyhal_gpio_write(PUMP_1_PIN, 1u);
				   vTaskDelay(PUMP1_ON_TIME);
				   cyhal_gpio_write(PUMP_1_PIN, 0u);

				   pump_action = NO_PUMP;
			   }

			   if(pump_action == PUMP_2)
			   {
				   cyhal_gpio_write(PUMP_2_PIN, 1u);
				   vTaskDelay(PUMP2_ON_TIME);
				   cyhal_gpio_write(PUMP_2_PIN, 0u);

				   pump_action = NO_PUMP;
			   }

			   if(pump_action == PUMP_3)
			   {
				   cyhal_gpio_write(PUMP_3_PIN, 1u);
				   vTaskDelay(PUMP3_ON_TIME);
				   cyhal_gpio_write(PUMP_3_PIN, 0u);

				   pump_action = NO_PUMP;
			   }

			   if(pump_action == PUMP_4)
			   {
				   cyhal_gpio_write(PUMP_4_PIN, 1u);
				   vTaskDelay(PUMP4_ON_TIME);
				   cyhal_gpio_write(PUMP_4_PIN, 0u);

				   pump_action = NO_PUMP;
			   }


            /* Initiate an asynchronous read operation. The event handler will be called
			 * when it is complete. */
			result = cyhal_adc_read_async_uv(&adc_obj, NUM_SCAN, result_arr);
			if(result != CY_RSLT_SUCCESS)
			{
				CY_ASSERT(0);
			}

			/*
			 * Read data from result list, input voltage in the result list is in
			 * microvolts. Convert it millivolts and print input voltage
			 *
			 */
			adc_result[0] = result_arr[CHANNEL_0] / MICRO_TO_MILLI_CONV_RATIO;
			adc_result[1] = result_arr[CHANNEL_1] / MICRO_TO_MILLI_CONV_RATIO;
			adc_result[2] = result_arr[CHANNEL_2] / MICRO_TO_MILLI_CONV_RATIO;
			adc_result[3] = result_arr[CHANNEL_3] / MICRO_TO_MILLI_CONV_RATIO;

			//APP_INFO(("Moisture Sns1:%d mV, Sns2:%d mV, Sns3:%d mV, Sns4:%d mV \n", adc_result[0],adc_result[1],adc_result[2],adc_result[3]));


			/* Clear async read complete flag */
				async_read_complete = false;

#ifdef ENABLE_TFT
        /* Display data on LCD */
		for (uint16_t i=0;i<4;i++)
		{
			sprintf(sensor_value_buffer, "Sensor %d:  %04d mV", (i+1),adc_result[i]);
			GUI_DispStringAt(sensor_value_buffer, SENSOR_DISPLAY_OFFSET, sensor_row_print+(ROW_OFFSET*i));
		}
#endif /* #ifdef ENABLE_TFT */


        if( http_event_stream != NULL )
        {
        	sprintf(sensor_value_buffer, "Moisture Sns1: %d mV, Sns2: %d mV, Sns3: %d mV, Sns4: %d mV \n", adc_result[0],adc_result[1],adc_result[2],adc_result[3]);

            strcpy(http_response, (char*)sensor_value_buffer);

            /* Add event stream header */
            result = cy_http_server_response_stream_write_payload( http_event_stream, (const void*)EVENT_STREAM_DATA, sizeof( EVENT_STREAM_DATA ) - 1);
            if(result != CY_RSLT_SUCCESS){
                ERR_INFO(("Updating event stream failed"));
                http_event_stream = NULL;
            }

            /* Message to client */
            result = cy_http_server_response_stream_write_payload( http_event_stream, http_response, sizeof(http_response) - 1);
            if(result != CY_RSLT_SUCCESS){
                ERR_INFO(("Updating event stream failed"));
                http_event_stream = NULL;
            }

            /* SSE is ended with two line feeds */
            result = cy_http_server_response_stream_write_payload( http_event_stream, (const void*)LFLF, sizeof( LFLF ) - 1);
            if(result != CY_RSLT_SUCCESS){
                ERR_INFO(("Updating event stream failed\r\n"));
                http_event_stream = NULL;
            }
        }

           vTaskDelay(pdMS_TO_TICKS(200));

        }

        if(SERVER_RECONFIGURE_REQUESTED == reconfiguration_request)
        {
            reconfigure_http_server();
            display_configuration();
            reconfiguration_request = SERVER_RECONFIGURED;

        }

    }

}

#ifdef ENABLE_TFT
/*******************************************************************************
* Function Name: initialize_display
********************************************************************************
* Summary:
*  Task that initializes the TFT display and sets the foreground and background
*  colour.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void initialize_display(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize TFT */
    result = GUI_Init();
    PRINT_AND_ASSERT(result, "Failed to initialize TFT\n");

    /* Set foreground and background color and font size */
    GUI_SetFont(GUI_FONT_13B_1);
    GUI_SetColor(GUI_WHITE);
    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
}
#endif /* #ifdef ENABLE_TFT */

/*******************************************************************************
* Function Name: display_configuration
********************************************************************************
* Summary:
*  Displays details for configuring device on serial terminal and TFT screen.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void display_configuration(void)
{
    cy_wcm_ip_address_t ip_address;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    uint32_t ip_addr;
    char display_ip_buffer[DISPLAY_BUFFER_LENGTH];
    char http_url[URL_LENGTH]={0};

	char display_buffer[SENSOR_BUFFER_LENGTH];

	/*Variable to store associated AP informations. */
	cy_wcm_associated_ap_info_t associated_ap_info;

	/* IP address of STA. */
	result = cy_wcm_get_ip_addr(CY_WCM_INTERFACE_TYPE_STA, &ip_address);
	PRINT_AND_ASSERT(result, "Failed to retrieveSoftAP IP address\n");

	/*Print message to connect to that ip address*/
	ip_addr = ip_address.ip.v4;
	sprintf(display_ip_buffer, "%u.%u.%u.%u",  (unsigned char) ( ( ip_addr >> 0 ) & 0xff ),
			(unsigned char) ( ( ip_addr >> 8 ) & 0xff ),
			(unsigned char) ( ( ip_addr >> 16 ) & 0xff ),
			(unsigned char) ( ( ip_addr >> 24 ) & 0xff ));

	sprintf(http_url, "http://%s:%d", display_ip_buffer, HTTP_PORT);

	/*Get the associated AP informations. */
	cy_wcm_get_associated_ap_info(&associated_ap_info);

	/* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
	APP_INFO(("\x1b[2J\x1b[;H"));
	APP_INFO(("******************************************************************\r\n"));
	APP_INFO(("On a device connected to the '%s' Wi-Fi network, \r\n", associated_ap_info.SSID));
	APP_INFO(("Open a web browser and go to : %s\r\n", http_url));
	APP_INFO(("Use the webpage to observe the sensor voltage.\r\n"));
	APP_INFO(("******************************************************************\r\n"));

	GUI_Clear();
	row = TOP_DISPLAY;
	GUI_SetFont(GUI_FONT_16B_1);
	GUI_DispString("***Wi-Fi Web Server - Sprinkler***");
	row += ROW_OFFSET;
	GUI_SetFont(GUI_FONT_13B_1);
	GUI_DispStringAt("On a device connected to the Wi-Fi network\r\n", 0, row);
	row += ROW_OFFSET;
	sprintf(display_buffer, "%s, \r\n", associated_ap_info.SSID);
	GUI_DispStringAt(display_buffer, 0, row);
	row += ROW_OFFSET;
	GUI_DispStringAt("open a web browser and go to : \r\n", 0, row);
	GUI_DispString(http_url);
	row += ROW_OFFSET;
	row += ROW_OFFSET;
	GUI_DispStringAt("Moisture Sensor Values: \r\n", 0, row);
	row += ROW_OFFSET;
	sensor_row_print = row;
}


/* [] END OF FILE */
