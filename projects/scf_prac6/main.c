/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>

/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* Launchpad, Wi-Fi and Sensors includes */
#include "msp432_launchpad_board.h"
#include "edu_boosterpack_microphone.h"
#include "edu_boosterpack_sensors.h"
#include "edu_boosterpack_rgb.h"
#include "edu_boosterpack_buzzer.h"
#include "cc3100_boosterpack.h"

/*----------------------------------------------------------------------------*/

#define USERNAME                ( "test")

#define SPAWN_TASK_PRIORITY     ( tskIDLE_PRIORITY + 7 )
#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 3 )
#define PARSER_TASK_PRIORITY    ( tskIDLE_PRIORITY + 2 )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define MAIN_STACK_SIZE         ( 2048 + 1024 )
#define BLINK_STACK_SIZE        ( 128 )


#define SPEECH_SERVER_ADDRESS   ( SL_IPV4_VAL(34,253,122,64) )
#define SERVER_ADDRESS          ( SL_IPV4_VAL(127,0,0,1) )
#define TCP_PORT_NUMBER         ( 5001 )


#define TX_BUFFER_SIZE          ( 512 )
#define RX_BUFFER_SIZE          ( 512 )

/*----------------------------------------------------------------------------*/

static SemaphoreHandle_t semaphore;
static SemaphoreHandle_t semphr_button;
/*----------------------------------------------------------------------------*/

static void BlinkTask(void *pvParameters);
static void microphone_interrupt(void);
static void ParserTask(void *pvParameters);

void buzzer_callback(void);
/*----------------------------------------------------------------------------*/

static void BlinkTask(void *pvParameters)
{
    while (true)
    {
        /* Turn red LED on */
        led_red_on();

        /* Sleep for 10 ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Turn red LED on */
        led_red_off();

        /* Sleep for 990 ms */
        vTaskDelay(pdMS_TO_TICKS(990));
    }
}

static void MainTask(void *pvParameters)
{
    SlSockAddrIn_t socket_addr;

    int32_t retVal = 0;
    int16_t tcp_data_socket = 0;

    /* Start the microphone */
    edu_boosterpack_microphone_init();
    edu_boosterpack_microphone_callback_set(microphone_interrupt);

    /* Restore Wi-Fi */
    retVal = wifi_restore();
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Initialize Wi-Fi */
    retVal = wifi_init();
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }
    while (1)
    {
        // Wait for a semaphore from the button
        if (xSemaphoreTake(semphr_button, portMAX_DELAY) == pdTRUE)
        {
            /* Create a TCP socket for data */
            wifi_set_socket_address(&socket_addr, SPEECH_SERVER_ADDRESS,
                                    TCP_PORT_NUMBER, true);
            tcp_data_socket = wifi_tcp_client_open(&socket_addr, false);
            if (tcp_data_socket < 0)
            {
                led_red_on();
                while (1)
                    ;
            }
            uint8_t data_buffer[64];
            uint16_t data_length;

            /* Write the username in JSON format */
            data_length = snprintf((char *) data_buffer, sizeof(data_buffer),
                                   "{\"username\": \"%s\"}", USERNAME);

            /* Send user information through TCP socket */
            retVal = wifi_tcp_client_send(tcp_data_socket, data_buffer,
                                          data_length);
            if (retVal < 0)
            {
                led_red_on();
                while (1)
                    ;
            }

            /* Start collecting samples from the microphone */
            edu_boosterpack_microphone_restart();
            edu_boosterpack_microphone_start();

            uint32_t start_time, current_time;
            int32_t elapsed_time;

            start_time = xTaskGetTickCount();

            bool is_finished = false;
            while (!is_finished)
            {
                /* Wait to be notified from interrupt */
                if (xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
                {
                    /* Turn green LED on */
                    led_green_on();

                    /* Stop collecting samples from the microphone */
                    edu_boosterpack_microphone_stop();

                    /* Get data from microphone */
                    uint8_t* data_buffer_ptr;
                    uint16_t data_buffer_len;
                    data_buffer_len = edu_boosterpack_microphone_get_data(
                            &data_buffer_ptr);

                    /* Send audio data through TCP socket */
                    retVal = wifi_tcp_client_send(tcp_data_socket,
                                                  data_buffer_ptr,
                                                  data_buffer_len);
                    if (retVal < 0)
                    {
                        led_red_on();
                        while (1)
                            ;
                    }

                    current_time = xTaskGetTickCount();
                    elapsed_time = (current_time - start_time)
                            / configTICK_RATE_HZ;

                    if (elapsed_time > 2)
                    {
                        is_finished = true;
                    }
                    else
                    {
                        edu_boosterpack_microphone_restart();
                        edu_boosterpack_microphone_start();
                    }

                    /* Turn green LED off */
                    led_green_off();
                }
            }

            /* Close TCP and UDP sockets */
            wifi_client_close(tcp_data_socket);
        }
    }
}

static void ParserTask(void *pvParameters)
{
    tune_t note = { E6, _M };

    SlSockAddrIn_t socket_addr;

    int32_t retVal = 0;
    int16_t tcp_socket = 0;

    uint8_t recv_buffer[16];
    uint8_t send_buffer[6] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };

    edu_boosterpack_rgb_init();

    // Init buzzer
    edu_boosterpack_buzzer_init();
    edu_boosterpack_set_bpm(78);
    edu_boosterpack_buzzer_callback_set(buzzer_callback);

    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Create a UDP socket */
    wifi_set_socket_address(&socket_addr, SERVER_ADDRESS, TCP_PORT_NUMBER,
                            true);

    tcp_socket = wifi_tcp_client_open(&socket_addr, false);

    if (tcp_socket < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    while (true)
    {
        retVal = wifi_tcp_client_receive(tcp_socket, recv_buffer, 16);
        if (retVal > 0)
        {
            switch (recv_buffer[0])
            {
            case 'g':
                edu_boosterpack_rgb_set(9, 255, 0);
                break;
            case 'r':
                edu_boosterpack_rgb_set(255, 0, 0);
                break;
            case 'z':
                edu_boosterpack_buzzer_play_tune(note, 1);
                break;
            case 'e': /* echo */
                wifi_tcp_client_send(tcp_socket, send_buffer, 6);
                break;
            }
        }

    }

}

void buzzer_callback(void)
{
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
}

/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal;

    /* Initialize the board */
    board_init();

    semphr_button = xSemaphoreCreateBinary();
    if (semphr_button == NULL)
    {
        led_red_on();
        while (1)
            ;
    }

    /*
     * Start the SimpleLink task
     */
    retVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /*
     * Create binary semaphore to
     */
    semaphore = xSemaphoreCreateBinary();
    if (semaphore == NULL)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Create blink task */
    retVal = xTaskCreate(BlinkTask, "BlinkTask",
                         BLINK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Create communication task */
    retVal = xTaskCreate(MainTask, "MainTask",
                MAIN_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL);
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Create communication task */
    retVal = xTaskCreate(ParserTask, "ParserTask",
                MAIN_STACK_SIZE,NULL, PARSER_TASK_PRIORITY,NULL);
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }


    /* Start the task scheduler */
    vTaskStartScheduler();

    return 0;
}

/*----------------------------------------------------------------------------*/

static void microphone_interrupt(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    /* Unblock the task by releasing the semaphore. */
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void PORT1_IRQHandler(void)
{
    uint32_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Lee el estado de la interrupcion generada por GPIO_PORT_P1
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);

    // Reset del flag de interrupcion del pin que la genera
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    // Chequea si la interrupcion la genero el pin P1.1
    if (status & GPIO_PIN1)
    {
        xSemaphoreGiveFromISR(semphr_button, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
