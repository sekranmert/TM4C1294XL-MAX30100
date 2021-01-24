
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// XDCtools Header files
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>
#include <ti/drivers/I2C.h>

#include "Board.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#define SOCKETTEST_IP     "192.168.2.6"
#define TASKSTACKSIZE     4096
#define OUTGOING_PORT     5011
#define INCOMING_PORT     5030

extern Mailbox_Handle mailbox0;
extern Semaphore_Handle semaphore2;


I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

uint32_t ctr=0 ;
Void Timer_ISR(UArg arg1)
{
    Semaphore_post(semaphore2);
}
Void SWI_ISR(UArg arg1)
{

    while(1){

        Semaphore_pend(semaphore2, BIOS_WAIT_FOREVER);
        System_printf("Counter %d\n", ctr);
        System_flush();
        ctr++;

    }
}
/*
 *
 *  ======== printError ========
 */
void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}

bool IIC_OpenComm(void)
{
    bool retval = false;

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;                 // minimum speed first
    i2c = I2C_open(Board_I2C0, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");   // error, retval is false
    }
    else {
        System_printf("I2C Initialized!\n");        // everything alright, retval is true
        retval = true;
    }
    System_flush();
    return retval;                                  // return true or false
}

void IIC_CloseComm(void)
{
    I2C_close(i2c);
    System_printf("IIC_CloseComm()   I2C closed!\n");
    System_flush();
}

bool IIC_writeReg(int device_ID, int addr, uint8_t val)
{
    uint8_t txBuffer[2];
    uint8_t rxBuffer[2];
    bool retval=false;

    // place parameters
    txBuffer[0] = addr;                             // register address
    txBuffer[1] = val;                              // value to be written
    i2cTransaction.slaveAddress = device_ID;        // device IIC ID
    i2cTransaction.writeBuf = txBuffer;             // buffer that holds the values to be written
    i2cTransaction.writeCount = 2;                  // 2 bytes will be sent
    i2cTransaction.readBuf = rxBuffer;              // receive buffer (in this case it is not used)
    i2cTransaction.readCount = 0;                   // no bytes to be received

    if (I2C_transfer(i2c, &i2cTransaction)) {       // start the transaction
        System_printf("IIC_writeReg(%d,%d)\n", addr, val);
        retval = true;                              // true will be returned to indicate that transaction is successful
    }
    else {
        System_printf("I2C Bus fault\n");           // there is an error, returns false
    }
    System_flush();

    return retval;
}

bool IIC_readReg(int device_ID, int addr, int no_of_bytes, char *buf)
{
    uint8_t txBuffer[2];
    bool retval=false;

    // addr: register number
    txBuffer[0] = addr;                             // 1 byte: register address
    i2cTransaction.slaveAddress = device_ID;        // Device Id
    i2cTransaction.writeBuf = txBuffer;             // buffer to be sent
    i2cTransaction.writeCount = 1;                  // send just register address
    i2cTransaction.readBuf = buf;                   // read into this buffer
    i2cTransaction.readCount = no_of_bytes;         // number of bytes to be read


    if (I2C_transfer(i2c, &i2cTransaction)) {
        //System_printf("IIC_readReg(%d,%d)\n", addr, buf[0]);
        retval=true;
    }
    else {
        System_printf("I2C Bus fault\n");
    }
    System_flush();

    return retval;
}

Void taskFxn(UArg arg0, UArg arg1)
{

    char buf[10];
    char BPMVal[8];
    int value;
    int cnt = 0;
    int pulseW = 0;
    int count = 0;
    int next1=0;
    IIC_OpenComm();
    int BPM;
    char mode;
    int pulStat = 0;
    int DC, DCOld,oldVal = 0;
    int BWOld,BWNew = 0;

    //Bus connection
    IIC_readReg(0x57, 0xFF, 1, buf);
    System_printf("Register 0xFF (WHO_AM_I) = 0x%x\n", buf[0]);
    System_flush();


    //Mode select
    IIC_readReg(0x57, 0x06, 1, buf);
    System_printf("prev mode is  = 0x%x\n", buf[0]);
    System_flush();
    mode = (buf[0] & 0xF8) | 0x02;
    IIC_writeReg(0x57, 0x06, mode);

    //Sampling Rate select
    IIC_readReg(0x57, 0x07, 1, buf);
    System_printf("prev sampling rate is  = 0x%x\n", buf[0]);
    System_flush();
    mode = (buf[0] & 0xE3) | (0x00<<2);
    IIC_writeReg(0x57,0x07,mode);

    //LED Pulse Width select
    IIC_readReg(0x57, 0x07, 1, buf);
    System_printf("prev pulse width is  = 0x%x\n", buf[0]);
    System_flush();
    mode = (buf[0] & 0xFC) | 0x03;
    IIC_writeReg(0x57,0x07,mode);

    //LED Current select

    mode = ( 0x08 << 4) | (0x0F) ;
    IIC_writeReg(0x57,0x09,mode);

    while(1) {
        IIC_readReg(0x57, 0x05, 4, buf);
        next1 = (buf[0] << 8) | buf[1];

        //dc filter//////////////////////////////////////
        DC = next1 + (0.75 * DCOld);
        next1 = DC - DCOld;
        DCOld = DC;
        value = next1;



        //butterworth filter//////////////////////////////
        BWNew = (2.452372752527856026e-1 * value) + (0.50952544949442879485 *BWOld);
        BWOld = BWNew;
        value = BWNew;

        Task_sleep(50);
        if(value > oldVal & pulStat == 0){
            pulStat = 1;

        }
        if(value <= oldVal-20 & pulStat == 1){

            pulseW =count;
            pulStat = 0;
            count = 0;
            System_printf("pulse\n");
            System_flush();

        }
        if (cnt==20){
            System_printf("BPM = %d\n", BPM);
            System_flush();
            cnt=0;
            sprintf(BPMVal, "%d", BPM);
            Mailbox_post(mailbox0, &BPM, BIOS_NO_WAIT);

        }
        cnt++;
        count++;
        oldVal = value;
        BPM = 1200 / pulseW;

    }

    IIC_CloseComm();
}


bool sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd, connStat, numSend;
    bool retval=false;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        close(sockfd);
        return false;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  // clear serverAddr structure
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     // convert port # to network order
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    connStat = connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("sendData2Server::Error while connecting to server\n");
    }
    else {
        numSend = send(sockfd, data, size, 0);       // send data to the server
        if(numSend < 0) {
            System_printf("sendData2Server::Error while sending data to server\n");
        }
        else {
            retval = true;      // we successfully sent the temperature string
        }
    }
    System_flush();
    close(sockfd);
    return retval;
}

Void clientSocketTask(UArg arg0, UArg arg1)
{   int BPM;
    char BPMVal[8];
    char string[128];

    while(1) {
        // wait for the semaphore that httpTask() will signal
        // when temperature string is retrieved from api.openweathermap.org site
        //

        Mailbox_pend(mailbox0, &BPM, BIOS_WAIT_FOREVER);
        sprintf(BPMVal, "%d", BPM);
        strcpy(string,"Heart Rate: ");
        strcat(string,BPMVal);
        if(BPM>90){
            strcat(string,", It's higher than normal\n");
        }
        else if(BPM<60){
            strcat(string,", It's lower than normal\n");
        }
        else{
            strcat(string,", It's normal\n");
        }



        // connect to SocketTest program on the system with given IP/port
        // send hello message whihc has a length of 5.
        //
        if(sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, string, strlen(string))) {
            System_printf("clientSocketTask:: Temperature is sent to the server\n");
            System_flush();
        }

    }
}

bool createTasks(void)
{
    static Task_Handle taskHandle1, taskHandle2, taskHandle3, taskHandle4, taskHandle5;
    Task_Params taskParams;
    Error_Block eb;

    Error_init(&eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle1 = Task_create((Task_FuncPtr)SWI_ISR, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle2 = Task_create((Task_FuncPtr)clientSocketTask, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle4 = Task_create((Task_FuncPtr)taskFxn, &taskParams, &eb);

    if (taskHandle1 == NULL ||  taskHandle2 == NULL || taskHandle4 == NULL) {
        printError("netIPAddrHook: Failed to create HTTP, Socket and Server Tasks\n", -1);
        return false;
    }

    return true;
}
//  This function is called when IP Addr is added or deleted
//
void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    // Create a HTTP task when the IP address is added
    if (fAdd) {
        createTasks();
    }
}

int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the HTTP GET example\nSystem provider is set to "
            "SysMin. Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();


    /* Start BIOS */
    BIOS_start();

    return (0);
}
