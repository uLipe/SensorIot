/*
 * @file main.c
 * @brief main application entry point
 * @author : THE JACK BAUER TEAM
 *
 */

#include <stdbool.h>
#include <string.h>
#include "MQTTClient.h"
#include <time.h>
#include <mraa.h>
#include <stdio.h>

#define ABS(x) (x < 0 ? -x : x)

/* Defines the number os messages to deposit */
#define NOOF_PUBLISH 128

/** Defines the tasks time slice */
#define ACC_SLICE       2
#define MQTT_SLICE      5

/* Define the network transfer */
static Network sensornw;
static Client  sensorClient;
static MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
static MessageData publish_message;

/** Forward references to thread funcs */
static void accTask(void *args );
static void mqttTask(void *args);
static void uartTask(void *args);
static bool mqttInit(void);
static bool accelInit(void);


typedef struct {
    int xpos;
    int ypos;
    int zpos;
    int tstamp;
}fixImu;

/* Register entry with the measured values */
typedef struct {
    //int posi_x;
    //int posi_y;

    char posi_x[8];
    char posi_y[8];

    //char velocity[8];

    //char timestamp[8];
}registry_t;

__timer_t t;

/* Topic name */
static const char topic[] = {"Sensor-Iot-Accel"};

/* BUffer for messaging */
static unsigned char msgBuff[256];
static unsigned char messageToTransmit[256];

/* global registry */
registry_t sensorData;
fixImu imu, prevImu;
int xvelocity[2] = {0,0};
int yvelocity[2]= {0,0};

int xposition[2]= {0,0};
int yposition[2]= {0,0};


/* timers for tasks */
unsigned int nowAcc = 0;
unsigned int nowMQTT = 0;

static bool mqttSend = false;

/* Serial device driver instance */
const char serial1[] = {"/dev/ttyMFD0"};
mraa_uart_context deviceSerial;


static void uartTask(void *args)
{
    int size;

    printf("[UART] Running UART Task \n\r");

    /* Gets data from uart queue */
    size = mraa_uart_read(deviceSerial, &imu, sizeof(imu));
    if(size != -1) {
        printf("[ACC] Leu com sucesso no hardware \n\r");
        printf("[ACC] x:%d, y:%d, z:%d, time:%d \n\r",
            imu.xpos, imu.ypos, imu.zpos, imu.tstamp);
    }
    else {
        printf("[ACC] Erro de escrita no hardware \n\r");
    }

    if((imu.xpos > -400) && (imu.xpos < 400)) {
        imu.xpos = 0;
    }

    if((imu.ypos > -350) && (imu.ypos < 350)) {
        imu.ypos = 0;
    }

}


/*
 * accTask()
 */
static void accTask(void *args )
{
    int xdelta, ydelta, zdelta;

    printf("[ACC] Running ACC Task \n\r");


    /* process the accel calculations here */
    xdelta = imu.xpos - prevImu.xpos;
    ydelta = imu.ypos - prevImu.ypos;
    zdelta = imu.zpos - prevImu.zpos;


    xvelocity[0] = xvelocity[1] + xdelta;
    yvelocity[0] = yvelocity[1] + ydelta;
    xposition[0] = xposition[1] + xvelocity[0];
    yposition[0] = yposition[1] + yvelocity[0];

    if(xposition[0] > 10000) xposition[0] = 10000;
    if(yposition[0] > 10000) yposition[0] = 10000;
    if(xposition[0] < 0) xposition[0] = 0;
    if(yposition[0] < 0) yposition[0] = 0;



    printf("[ACC]xpos: %d, ypos: %d \n\r", xposition[0], yposition[0]);

    /*Prepare data to mqtt task */
    sprintf(&sensorData.posi_x, "%d", (xposition[0]/3));
    sprintf(&sensorData.posi_y, "%d", (yposition[0]/3));

    /*update calculation history */
    memcpy(&prevImu, &imu, sizeof(prevImu));

    if(ABS(xvelocity[0] - xvelocity[1]) >= 20) {
        xvelocity[1] = xvelocity[0] = 0;
    }

    if(ABS(xposition[0] - xposition[1]) >= 200) {
        xposition[1] = xposition[0];
    }

    if(ABS(yvelocity[0] - yvelocity[1]) >= 20) {
        yvelocity[1] = yvelocity[0] = 0;
    }

    if(ABS(yposition[0] - yposition[1]) >= 200) {
        yposition[1] = yposition[0];
    }
/*
    xvelocity[1] = xvelocity[0];
    xposition[1] = xposition[0];
    yvelocity[1] = yvelocity[0];
    yposition[1] = yposition[0];
 */

}

/*
 * mqttTask()
 */
static void mqttTask(void *args)
{
    int rc = -1;

    printf("[MQTT] Running MQTT Task \n\r");

    /* Mount message frame */
    publish_message.message->qos = QOS2;
    publish_message.message->retained = true;
    publish_message.message->dup = false;
    publish_message.message->payload = (void *)&sensorData;
    publish_message.message->payloadlen = sizeof(sensorData);


    if(mqttSend != false ) {
            rc = MQTTPublish(&sensorClient, (const char *)&topic[0] ,publish_message.message);
            if(rc != 0) {
                printf("Message transmission failed \n\r");
            }
            else {
                printf("Message transmission passed \n\r");
            }
            mqttSend = false;
    }

}

/*
 * mqttTask()
 */
 static bool mqttInit(void)
 {
     bool ret = false;
     int rc = -1;

     memset(&messageToTransmit,0, sizeof(messageToTransmit));
     memset(&publish_message,0, sizeof(publish_message));

     /* Allocate the fields for message population */
     publish_message.message = (MessageData *)malloc(sizeof(MQTTMessage));
     if(publish_message.message == NULL) {
         printf("[MQTTTRACE] ponteiro nulo \n\r");
         while(1);
     }

     publish_message.topicName = (MQTTString *)malloc(32);

     /* Prefixes the topic name */
     strncpy(publish_message.topicName, &topic[0], sizeof(topic));

     /** Alloc the net interface */
     NewNetwork(&sensornw);
     rc = ConnectNetwork(&sensornw, "iot.eclipse.org", 1883);

     /* Check for errors */
     if(rc != 0){
         printf("Error, cannot connect to network \n\r");
     }
     else {

         printf("Connection accepted! \n\r");

         /* So if net were open, prepare mqtt client */
         MQTTClient(&sensorClient, &sensornw, 1000, messageToTransmit,
                     256, msgBuff, 256);

         /* Initialized tries to connect via mqtt */
         data.willFlag = 0;
         data.MQTTVersion = 3;
         data.clientID.cstring = "      ";
         data.username.cstring = "      ";
         data.password.cstring = "      ";
         data.keepAliveInterval = 60;
         data.cleansession = 1;
         rc = MQTTConnect(&sensorClient, &data);
         if(rc != 0 ){
             printf ("MQTT connection refused! \n\r");
             sensornw.disconnect(&sensornw);

         }
         else {
             int i = 0;
             /* Connection fully accepted: */
             printf ("Horray! MQTT connected! \n\r");
             ret = true;
         }
     }
     return(ret);
 }

 /*
  *
  */
static bool accelInit(void)
{
    bool ret = true ;

    memset(&imu, 0, sizeof(imu));
    deviceSerial = mraa_uart_init(0);

    if(deviceSerial != NULL) {
        printf("[ACC] Abriu a porta serial corretamente \n\r");
        mraa_uart_set_baudrate(deviceSerial, 115200);
        //printf("[ACC] Handler da serial %d \n\r", deviceSerial);
    }
    else {
        printf("[ACC] Nao abriu a porta serial corretamente \n\r");
    }
    sleep(2);
    return(ret);
}

/*
 * main()
 */
int main (int argc, char *argv[])
{
    memset(&sensorData,0, sizeof(sensorData));

    /* try to up the mqtt client */
    if(mqttInit()!=true) {
        printf("[MQTT] Mqtt failed to connect \n\r");
    }
    else {
        printf("[MQTT] Mqtt connected and is ready for use \n\r");
    }

    /* now try to up the IMU sensors */
    if(accelInit()!=true) {
        printf("[ACC] IMU sensors with error \n\r");
    }
    else {
        printf("[ACC] IMU sensors are up and ready \n\r");
    }

    /*
     * Create and fire the threads:
     */
     while(1) {

         /* Always run the uart rcv task */
         uartTask(NULL);

         if(time(&t) - nowAcc >= ACC_SLICE ){
            /* time to run acc task */
             nowAcc = time(&t);
             accTask(NULL);
         }

         /* check for time to send packet */
         if(time(&t) - nowMQTT >= (MQTT_SLICE)){
             mqttSend = true;
             nowMQTT = time(&t);
             mqttTask(0);
         }

     }
    return 0;
}
