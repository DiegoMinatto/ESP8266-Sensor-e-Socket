/*
	Autor: Prof. Vagner Rodrigues
	Objetivo: Manipulação das GPIOs utilizando SDK-IDF com RTOS (FreeRTOS)
			  Controlando GPIO por Tasks
	Disciplina: IoT II
	Curso: Engenharia da Computação
*/

/*	Relação entre pinos da WeMos D1 R2 e GPIOs do ESP8266
	Pinos-WeMos		Função			Pino-ESP-8266
		TX			TXD				TXD/GPIO1
		RX			RXD				RXD/GPIO3
		D0			IO				GPIO16	
		D1			IO, SCL			GPIO5
		D2			IO, SDA			GPIO4
		D3			IO, 10k PU		GPIO0
		D4			IO, 10k PU, LED GPIO2
		D5			IO, SCK			GPIO14
		D6			IO, MISO		GPIO12
		D7			IO, MOSI		GPIO13
		D0			IO, 10k PD, SS	GPIO15
		A0			Inp. AN 3,3Vmax	A0
*/

/* Inclusão das Bibliotecas */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include <lwip/netdb.h>

#include "lwip/sockets.h"

#include <dht.h>

#include <stdbool.h>
#include <ultrasonic.h>

static const dht_sensor_type_t sensor_type = DHT_TYPE_DHT11;
#if defined(CONFIG_IDF_TARGET_ESP8266)
static const gpio_num_t dht_gpio = 5;
#else
static const gpio_num_t dht_gpio = 17;
#endif


#define MAX_DISTANCE_CM 500 // 5m max
#define TRIGGER_GPIO 0
#define ECHO_GPIO 4

QueueHandle_t bufferTemp;
QueueHandle_t bufferDist;

/* Definições e Constantes */
#define TRUE  1
#define FALSE 0
#define DEBUG TRUE 
#define LED_BUILDING	GPIO_NUM_2 
#define BUTTON			GPIO_NUM_14 
#define LED_EXTERNA     GPIO_NUM_5
#define GPIO_INPUT_PIN_SEL  	(1ULL<<BUTTON)
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#define PORT CONFIG_EXAMPLE_PORT
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_CONNECTING_BIT BIT2
#define BUTTON_BIT BIT0


/* Protótipos de Funções */
void app_main( void );
void task_GPIO_Blink( void *pvParameter );
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void task_ip( void *pvParameter );
static void wifi_init_sta(  void  );

/* Variáveis Globais */
static const char * TAG = "main: ";
const char * msg[2] = {"Ligado", "Desligado"};
static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t button_event_group;
static int s_retry_num = 0;
static bool btnApertado = false;
 
void task_GPIO_Blink( void *pvParameter )
{
	 /*  Parâmetros de controle da GPIO da função "gpio_set_direction"
		GPIO_MODE_OUTPUT       			//Saída
		GPIO_MODE_INPUT        			//Entrada
		GPIO_MODE_INPUT_OUTPUT 			//Dreno Aberto
    */
	bool inverte = true;
	int delay = 100;
	gpio_set_direction( LED_BUILDING, GPIO_MODE_OUTPUT );
	gpio_set_level( LED_BUILDING, 1 );  //O Led desliga em nível 1;
    bool estado = 0;   

    while ( TRUE ) 
    {	

        //xEventGroupWaitBits(button_event_group, BUTTON_BIT, pdFALSE, pdFALSE, portMAX_DELAY);	
      EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                             WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_CONNECTING_BIT,
                                             pdFALSE,
                                             pdFALSE,
                                             portMAX_DELAY);
										
      /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
       * happened. */
      if (bits & WIFI_CONNECTED_BIT) {
        inverte = false;
		estado = false;
		delay = 100;		 
      } else if (bits & WIFI_FAIL_BIT) {
        inverte = true;
		delay = 100;
      }else if(bits & WIFI_CONNECTING_BIT){
		  inverte = true;
		  delay = 500;
	  }
	  if(inverte){
        estado = !estado;
	  }
      gpio_set_level( LED_BUILDING, estado ); 				
      vTaskDelay( delay / portTICK_RATE_MS ); //Delay de 2000ms liberando scheduler;
	}
}	

/*
  Função de callback responsável em receber as notificações durante as etapas de conexão do WiFi.
  Por meio desta função de callback podemos saber o momento em que o WiFi do ESP8266 foi inicializado com sucesso
  até quando é recebido o aceite do IP pelo roteador (no caso de Ip dinâmico).
  ref: https://github.com/espressif/esp-idf/tree/c77c4ccf6c43ab09fd89e7c907bf5cf2a3499e3b/examples/wifi/getting_started/station
*/
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		if( DEBUG )
		    ESP_LOGI(TAG, "Tentando conectar ao WiFi...\r\n");
		    
		/*
			O WiFi do ESP8266 foi configurado com sucesso. 
			Agora precisamos conectar a rede WiFi local. Portanto, foi chamado a função esp_wifi_connect();
		*/
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
			xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTING_BIT);
			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
			xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
			/*
			Se chegou aqui foi devido a falha de conexão com a rede WiFi.
			Por esse motivo, haverá uma nova tentativa de conexão WiFi pelo ESP8266.
			*/
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Tentando reconectar ao WiFi...");
        } else {
			/*
				É necessário apagar o bit para avisar as demais Tasks que a 
				conexão WiFi está offline no momento. 
			*/
			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTING_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Falha ao conectar ao WiFi");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		/*
			Conexão efetuada com sucesso. Busca e imprime o IP atribuido. 
		*/
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado! O IP atribuido é:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
		/*
				Seta o bit indicativo para avisar as demais Tasks que o WiFi foi conectado. 
		*/
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

 /* Inicializa o WiFi em modo cliente (Station) */
void wifi_init_sta(void)
{
   

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    /* Setting a password implies station will connect to all security modes including WEP/WPA.
        * However these modes are deprecated and not advisable to be used. Incase your Access point
        * doesn't support WPA2, these mode can be enabled by commenting below line */

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Conectado ao AP SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Falha ao conectar ao AP SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "EVENTO INESPERADO");
    }

   // ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
  //  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    //vEventGroupDelete(s_wifi_event_group);
}

void task_ip( void *pvParameter )
{
    
    if( DEBUG )
      ESP_LOGI( TAG, "Inicializada task_ip...\r\n" );
	
    while (TRUE) 
    {    
		/* o EventGroup bloqueia a task enquanto a rede WiFi não for configurada */
		xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);	
    
		/* Após configurada a rede WiFi recebemos e imprimimos a informação do IP atribuido */
		tcpip_adapter_ip_info_t ip_info;
	    ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));  
		
		/* Imprime o IP, bloqueia a task e repete a a impressão a cada 5 segundos */
    	if( DEBUG )
      		ESP_LOGI( TAG, "IP atribuido:  %s\n", ip4addr_ntoa(&ip_info.ip) );
		vTaskDelay( 5000/portTICK_PERIOD_MS );
    }
}

void task_Button(void *pvParameter){
	gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON,GPIO_PULLUP_ONLY);
	while(TRUE){
		if(!gpio_get_level(BUTTON)){
			if(!btnApertado){ 
			  gpio_set_level(LED_EXTERNA, 1); 
			  btnApertado = true;
			  xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
              s_retry_num = 0;
			  gpio_set_level(LED_EXTERNA, 0); 
              esp_wifi_connect();
			  xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
			}
		}else{
		  btnApertado = false;
		}
		vTaskDelay(50/portTICK_RATE_MS);
	}
}

static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
///////////////////////////////////////////////////////////////////////////////
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
///////////////////////////////////////////////////////////////////////////////
        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            //break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            //break;
        }
        ESP_LOGI(TAG, "Socket binded");
        while (1) {
        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

//////////////////////////////////////////////
        struct sockaddr_in sourceAddr;
//////////////////////////////////////////////
        uint addrLen = sizeof(sourceAddr);
        int sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");
        char rcv_temp[32];
		char rcv_dist[32];
		char envioa[1000];
		char enviob[1000];
        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {

                ///////////////////////////////////////////////////
                inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                //////////////////////////////////////////////////

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
				
				if(len == 4 && rx_buffer[0] == 'T' && rx_buffer[1] == 'E' && rx_buffer[2] == 'M' && rx_buffer[3] == 'P' ){
					
				                   
		          if (xQueueReceive(bufferTemp, &rcv_temp, pdMS_TO_TICKS(1000)) == true){                   
                    ESP_LOGI( TAG, rcv_temp);
					sprintf(envioa,"%s%s%s",rcv_temp,"C","\r\n");
				    int err = send(sock, envioa, strlen(envioa), 0);
                    if (err < 0) {
                      ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                      break;
                    }
                  }else{
                    ESP_LOGE("Queue", "Item nao recebido, timeout expirou!");//Se o timeout expirou, mostra erro
					break;
                  }
                  
				}else{
					
				  if(len == 4 && rx_buffer[0] == 'D' && rx_buffer[1] == 'I' && rx_buffer[2] == 'S' && rx_buffer[3] == 'T' ){
					
				                   
		            if (xQueueReceive(bufferDist, &rcv_dist, pdMS_TO_TICKS(1000)) == true){                   
                      ESP_LOGI( TAG, rcv_dist);
					  sprintf(enviob,"%s%s%s",rcv_dist,"Cm","\r\n");
				      int err = send(sock, enviob, strlen(enviob), 0);
                      if (err < 0) {
                        ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                        break;
                      }
                    }else{
                      ESP_LOGE("Queue", "Item nao recebido, timeout expirou!");//Se o timeout expirou, mostra erro
				      break;
                    }
                  
				  }else{
				    int err = send(sock, "Comandos diponiveis: TEMP, DIST\r\n", strlen("Comandos diponiveis: TEMP, DIST\r\n"), 0);
                    if (err < 0) {
                     ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                     break;
                    }
				  }						  
				}		
				
				
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

               
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void leituraSensorTemp(void *pvParameters)
{
    int16_t temperature = 0;
    int16_t humidity = 0;
	char integer_string[32];
    while(1)
    {

        if (dht_read_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK){
		sprintf(integer_string, "%d", (temperature / 10));
				
		xQueueSend(bufferTemp, strcat(integer_string,"C"), pdMS_TO_TICKS(0));
        }else{
			xQueueSend(bufferTemp, "0", pdMS_TO_TICKS(0));
			ESP_LOGE(TAG, "Erro ao ler o sensor!");
		}
            
        vTaskDelay(2000 / portTICK_PERIOD_MS);

    }
}

void ultrasonic_test(void *pvParamters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);
    char integer_string[32];
	xQueueSend(bufferDist, "0", pdMS_TO_TICKS(0));
    while (true)
    {
        uint32_t distance;
        esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error: ");
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%d\n", res);
            }
        }
        else{
			sprintf(integer_string, "%d", distance);
			xQueueSend(bufferDist, integer_string, pdMS_TO_TICKS(0));
			ESP_LOGI( TAG, integer_string);
		}
            //printf("Distance: %d cm\n", distance);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/* Aplicação Principal (Inicia após bootloader) */
void app_main( void )
{
	bufferTemp = xQueueCreate(1, sizeof(32));
	bufferDist = xQueueCreate(1, sizeof(32));
	gpio_set_direction( LED_EXTERNA, GPIO_MODE_OUTPUT );
    button_event_group = xEventGroupCreate();	
	 s_wifi_event_group = xEventGroupCreate(); //Cria o grupo de eventos
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	
	
  
		
		// Cria a task responsável pelo blink LED. 
	if( (xTaskCreate( task_GPIO_Blink, "task_GPIO_Blink", 2048, NULL, 2, NULL )) != pdTRUE )
    {
      if( DEBUG )
        ESP_LOGI( TAG, "error - Nao foi possivel alocar task_GPIO_Blink.\r\n" );  
      return;   
    } 
	//Configura e inicializa o WiFi.
    wifi_init_sta();	
	// Cria a task responsável por imprimir o IP recebido do roteador.
    if(xTaskCreate( task_ip, "task_ip", 2048, NULL, 1, NULL )!= pdTRUE )
	{
		if( DEBUG )
			ESP_LOGI( TAG, "error - nao foi possivel alocar Task_IP.\n" );	
		return;		
	}
	// Cria a task responsável pelo blink LED. 
	if( (xTaskCreate( task_Button, "task_Button", 2048, NULL, 5, NULL )) != pdTRUE )
    {
      if( DEBUG )
        ESP_LOGI( TAG, "error - Nao foi possivel alocar task_Button.\r\n" );  
      return;   
    }  
	if( (xTaskCreate(ultrasonic_test, "ultrasonic_test", 4096, NULL, 5, NULL)) != pdTRUE )
    {
      if( DEBUG )
        ESP_LOGI( TAG, "error - Nao foi possivel alocar ultrasonic_test.\r\n" );  
      return;   
    }	
	if( (xTaskCreate(leituraSensorTemp, "leituraSensorTemp", 4096, NULL, 1, NULL)) != pdTRUE )
    {
      if( DEBUG )
        ESP_LOGI( TAG, "error - Nao foi possivel alocar leituraSensorTemp.\r\n" );  
      return;   
    }
		if( (xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL)) != pdTRUE )
    {
      if( DEBUG )
        ESP_LOGI( TAG, "error - Nao foi possivel alocar tcp_server_task.\r\n" );  
      return;   
    }  
	  
	
	

}