/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include <stdbool.h>

/* USER CODE BEGIN Includes */
//#include "sx126x-hal.h"
#include "sx126x.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

#define SEND_PING_BEAT_US 500000
#define RX_TIMEOUT_US 500000

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1
#define ADV_DEBUG 1

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    868500000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

//#if USE_MODEM_LORA == 1

    #define LORA_BANDWIDTH                              LORA_BW_125         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       LORA_SF7         // [SF7..SF12]
    #define LORA_LOWDATARATEOPTIMIZE                    0
    #define LORA_CODINGRATE                             LORA_CR_4_5         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_HEADER_TYPE                            LORA_PACKET_VARIABLE_LENGTH
    #define LORA_FHSS_ENABLED                           false
    #define LORA_NB_SYMB_HOP                            4
    #define LORA_IQ                                     LORA_IQ_NORMAL
    #define LORA_CRC_MODE                               LORA_CRC_ON

/*#elif USE_MODEM_FSK == 1

    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               RX_BW_93800     // Hz
    #define FSK_MODULATION_SHAPPING                     MOD_SHAPING_G_BT_05
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_HEADER_TYPE                             RADIO_PACKET_VARIABLE_LENGTH
    #define FSK_CRC_MODE                                RADIO_CRC_2_BYTES_CCIT
    #define FSK_ADDR_FILTERING                          RADIO_ADDRESSCOMP_FILT_NODE;
    #define FSK_WHITENING_MODE                          RADIO_DC_FREE_OFF
    #define FSK_PREAMBLE_DETECTOR_MODE                  RADIO_PREAMBLE_DETECTOR_OFF
    #define FSK_SYNCWORD_LENGTH                         8
#else
    #error "Please define a modem in the compiler options."
#endif*/

#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     32        // Define the payload size here

 /*
 * Callback functions prototypes
 */
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * @brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t errCode );

/*!
 * @brief Function executed on Radio Fhss Change Channel event
 */
void OnFhssChangeChannel( uint8_t channelIndex );

typedef struct{
    RadioPacketTypes_t packetType;
    int8_t txPower;
    RadioRampTimes_t txRampTime;
    ModulationParams_t modParams;
    PacketParams_t packetParams;
    uint32_t rfFrequency;
    uint16_t irqTx;
    uint16_t irqRx;
    uint32_t txTimeout;
    uint32_t rxTimeout;
}RadioConfigurations_t;
RadioConfigurations_t radioConfiguration;

/*
 *  Global variables declarations
 */
typedef enum
{
    SEND_PACKET,
    WAIT_SEND_DONE,
    RECEIVE_PACKET,
    WAIT_RECEIVE_DONE,
    PACKET_RECEIVED,
}AppStates_t;
volatile AppStates_t State = SEND_PACKET;

typedef struct{
    bool rxDone;
    bool rxError;
    bool txDone;
    bool rxTimeout;
    bool txTimeout;
}RadioFlags_t;
RadioFlags_t radioFlags = {
    .txDone = false,
    .rxDone = false,
    .rxError = false,
    .rxTimeout = false,
    .txTimeout = false,
};

/*!
 * Radio events function pointer
 */
static RadioCallbacks_t RadioEvents = {
    .txDone = &OnTxDone,
    .txTimeout = &OnTxTimeout,
    .rxDone = &OnRxDone,
    .rxPreambleDetect = NULL,
    .rxHeaderDone = NULL,
    .rxTimeout = &OnRxTimeout,
    .rxError = &OnRxError,
    .cadDone = NULL,
};

/*
 *  Global variables declarations
 */
//Radio Radio( NULL );
#define MESSAGE_SIZE 4
typedef uint8_t Messages_t[MESSAGE_SIZE];
const Messages_t PingMsg = {'P', 'I', 'N', 'G'};
const Messages_t PongMsg = {'P', 'O', 'N', 'G'};
const Messages_t *messageToReceive = &PongMsg;
const Messages_t *messageToSend = &PingMsg;

uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int8_t RssiValue = 0;
int8_t SnrValue = 0;

static bool ImageCalibrated = false;
static uint8_t OPT = 1;


RadioOperatingModes_t OperatingMode;

void GetRssiSnr(int8_t *rssi, int8_t *snr);
//SX126xHal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, A1, A2, D8, &RadioEvents );

void SetToMaster(void);
void SetToSlave(void);
void RunMasterStateMachine();
void RunSlaveStateMachine();
void SetConfiguration(RadioConfigurations_t *config);
//void ConfigureGeneralRadio(SX126xHal *radio, RadioConfigurations_t *config);
//void ConfigureRadioTx(SX126xHal *radio, RadioConfigurations_t *config);
//void ConfigureRadioRx(SX126xHal *radio, RadioConfigurations_t *config);
//void PrepareBuffer(SX126xHal *radio, const Messages_t *messageToSend);
bool isMaster = true;
bool masterCanSend = true;
void MasterSendNextEvent(void){masterCanSend = true;}
bool slaveCanListen = false;
void SlaveListenNextEvent(void){slaveCanListen = true;}
//Ticker masterSendNextTicker;
//Ticker slaveListenNextTicker;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();


  /* USER CODE BEGIN 2 */
  sx126x_Reset();
  sx126x_Init();
  //serial.baud(115200);
  SetToMaster();
  SetConfiguration(&radioConfiguration);
  ConfigureGeneralRadio(&radioConfiguration);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */


  /* USER CODE BEGIN 3 */
    //if(isMaster){
     //RunMasterStateMachine();
    //}

    PrepareBuffer(messageToSend);
    ConfigureRadioTx(&radioConfiguration);
    sx126x_SetTx(radioConfiguration.txTimeout);

    HAL_Delay(500);

  }
  /* USER CODE END 3 */

}

void ConfigureRadioTx(RadioConfigurations_t *config){
    sx126x_SetDioIrqParams(config->irqTx, config->irqTx, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

void PrepareBuffer(const Messages_t *messageToSend){
    sx126x_SetPayload((uint8_t*)messageToSend, MESSAGE_SIZE);
}

void sx126x_SetTx( uint32_t timeout )
{
    uint8_t buf[3];

    OperatingMode = MODE_TX;

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    sx126x_WriteCommand( RADIO_SET_TX, buf, 3 );
}

void sx126x_SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];


    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    sx126x_WriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}

void sx126x_SetPayload( uint8_t *payload, uint8_t size )
{
    sx126x_WriteBuffer( 0x00, payload, size );
}


void SetToMaster(){
    //printf("-->Master\n");
    isMaster = true;
    masterCanSend = true;
    State = SEND_PACKET;
    messageToReceive = &PongMsg;
    messageToSend = &PingMsg;
}


void ConfigureGeneralRadio(RadioConfigurations_t *config){
    sx126x_SetPacketType(config->packetType);
    sx126x_SetPacketParams(&config->packetParams);
    sx126x_SetModulationParams(&config->modParams);
    sx126x_SetRfFrequency(config->rfFrequency);
    sx126x_SetTxParams(config->txPower, config->txRampTime);
    //sx126x_SetInterruptMode();
    if(config->packetType == PACKET_TYPE_GFSK){
        uint8_t syncword[8] = {0xF0, 0x0F, 0x55, 0xAA, 0xF0, 0x0F, 0x55, 0xAA};
        sx126x_SetSyncWord(syncword);
    }
}

void sx126x_SetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];
    //DigitalIn OPT( A3 );

    //if( GetDeviceType( ) == SX1261 )
    //{
        if( power == 15 )
        {
            sx126x_SetPaConfig( 0x06, 0x00, 0x01, 0x01 );
        }
        else
        {
            sx126x_SetPaConfig( 0x04, 0x00, 0x01, 0x01 );
        }
        if( power >= 14 )
        {
            power = 14;
        }
        else if( power < -3 )
        {
            power = -3;
        }
        sx126x_WriteReg( REG_OCP, 0x18, 1 ); // current max is 80 mA for the whole device
    //}
    /*else // sx1262 or sx1268
    {
        SetPaConfig( 0x04, 0x07, 0x00, 0x01 );
        if( power > 22 )
        {
            power = 22;
        }
        else if( power < -3 )
        {
            power = -3;
        }
        WriteReg( REG_OCP, 0x38 ); // current max 160mA for the whole device
    }*/
    buf[0] = power;
    if( OPT == 0 )
    {
        if( ( uint8_t )rampTime < RADIO_RAMP_200_US )
        {
            buf[1] = RADIO_RAMP_200_US;
        }
        else
        {
            buf[1] = ( uint8_t )rampTime;
        }
    }
    else
    {
        buf[1] = ( uint8_t )rampTime;
    }
    sx126x_WriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

void sx126x_SetPaConfig( uint8_t paDutyCycle, uint8_t HpMax, uint8_t deviceSel, uint8_t paLUT )
{
    uint8_t buf[4];


    buf[0] = paDutyCycle;
    buf[1] = HpMax;
    buf[2] = deviceSel;
    buf[3] = paLUT;
    sx126x_WriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}

void sx126x_SetRfFrequency( uint32_t frequency )
{
    uint8_t buf[4];
    uint32_t freq = 0;


    if( ImageCalibrated == false )
    {
        sx126x_CalibrateImage( frequency );
        ImageCalibrated = true;
    }

    freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );
    sx126x_WriteCommand( RADIO_SET_RFFREQUENCY, buf, 4 );
}


void sx126x_CalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xD8;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    sx126x_WriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}

void sx126x_SetModulationParams( ModulationParams_t *modulationParams )
{
    uint8_t n;
    uint32_t tempVal = 0;
    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    /*if( this->PacketType != modulationParams->PacketType )
    {
        this->SetPacketType( modulationParams->PacketType );
    }*/

    switch( modulationParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        n = 8;
        tempVal = ( uint32_t )( 32 * ( ( double )XTAL_FREQ / ( double )modulationParams->Params.Gfsk.BitRate ) );
        buf[0] = ( tempVal >> 16 ) & 0xFF;
        buf[1] = ( tempVal >> 8 ) & 0xFF;
        buf[2] = tempVal & 0xFF;
        buf[3] = modulationParams->Params.Gfsk.ModulationShaping;
        buf[4] = modulationParams->Params.Gfsk.Bandwidth;
        tempVal = ( uint32_t )( ( double )modulationParams->Params.Gfsk.Fdev / ( double )FREQ_STEP );
        buf[5] = ( tempVal >> 16 ) & 0xFF;
        buf[6] = ( tempVal >> 8 ) & 0xFF;
        buf[7] = ( tempVal& 0xFF );
        break;
    case PACKET_TYPE_LORA:
        n = 4;
        switch( modulationParams->Params.LoRa.Bandwidth )
        {
            case LORA_BW_500:
                 modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                break;
            case LORA_BW_250:
                if( modulationParams->Params.LoRa.SpreadingFactor == 12 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_125:
                if( modulationParams->Params.LoRa.SpreadingFactor >= 11 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_062:
                if( modulationParams->Params.LoRa.SpreadingFactor >= 10 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_041:
                if( modulationParams->Params.LoRa.SpreadingFactor >= 9 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_031:
            case LORA_BW_020:
            case LORA_BW_015:
            case LORA_BW_010:
            case LORA_BW_007:
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                break;
            default:
                break;
        }
        buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
        buf[1] = modulationParams->Params.LoRa.Bandwidth;
        buf[2] = modulationParams->Params.LoRa.CodingRate;
        buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;
        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
    sx126x_WriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
}

void sx126x_SetSyncWord( uint8_t *syncWord )
{
    sx126x_WriteReg( REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 );
    //return 0;
}


void sx126x_SetPacketParams( PacketParams_t *packetParams )
{
    uint8_t n;
    uint8_t crcVal = 0;
    uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    /*if( this->PacketType != packetParams->PacketType )
    {
        this->SetPacketType( packetParams->PacketType );
    }*/

    //sx126x_SetPacketType( packetParams->PacketType );

    switch( packetParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM )
        {
            sx126x_SetCrcSeed( CRC_IBM_SEED );
            sx126x_SetCrcPolynomial( CRC_POLYNOMIAL_IBM );
            crcVal = RADIO_CRC_2_BYTES;
        }
        else if(  packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT )
        {
            sx126x_SetCrcSeed( CRC_CCITT_SEED );
            sx126x_SetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
            crcVal = RADIO_CRC_2_BYTES_INV;
        }
        else
        {
            crcVal = packetParams->Params.Gfsk.CrcLength;
        }
        n = 9;
        // convert preamble length from byte to bit
        packetParams->Params.Gfsk.PreambleLength = packetParams->Params.Gfsk.PreambleLength << 3;

        buf[0] = ( packetParams->Params.Gfsk.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.Gfsk.PreambleLength;
        buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
        buf[3] = ( packetParams->Params.Gfsk.SyncWordLength << 3 ); // convert from byte to bit
        buf[4] = packetParams->Params.Gfsk.AddrComp;
        buf[5] = packetParams->Params.Gfsk.HeaderType;
        buf[6] = packetParams->Params.Gfsk.PayloadLength;
        buf[7] = crcVal;
        buf[8] = packetParams->Params.Gfsk.DcFree;
        break;
    case PACKET_TYPE_LORA:
        n = 6;
        buf[0] = ( packetParams->Params.LoRa.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.LoRa.PreambleLength;
        buf[2] = packetParams->Params.LoRa.HeaderType;
        buf[3] = packetParams->Params.LoRa.PayloadLength;
        buf[4] = packetParams->Params.LoRa.CrcMode;
        buf[5] = packetParams->Params.LoRa.InvertIQ;
        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
    sx126x_WriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
}


void sx126x_SetCrcSeed( uint16_t seed )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( seed >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( seed & 0xFF );

    if (USE_MODEM_LORA == 0)
    {
        sx126x_WriteRegister( REG_LR_CRCSEEDBASEADDR, buf, 2 );
        return;
    } else {
        return;
    }
}

void sx126x_SetCrcPolynomial( uint16_t polynomial )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( polynomial >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( polynomial & 0xFF );

    if (USE_MODEM_LORA == 0)
    {
        sx126x_WriteRegister( REG_LR_CRCPOLYBASEADDR, buf, 2 );
        return;
    } else {
        return;
    }
}

void sx126x_SetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    //this->PacketType = packetType;
    sx126x_WriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}


void SetConfiguration(RadioConfigurations_t *config){
    config->irqRx = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
    config->irqTx = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    config->rfFrequency = RF_FREQUENCY;
    config->txTimeout = 0;
    config->rxTimeout = (uint32_t)(RX_TIMEOUT_US / 15.625);
    config->txPower = TX_OUTPUT_POWER;
    config->txRampTime = RADIO_RAMP_200_US;
    #if USE_MODEM_LORA == 1
        config->packetType = PACKET_TYPE_LORA;
        config->modParams.PacketType = PACKET_TYPE_LORA;
        config->modParams.Params.LoRa.Bandwidth = LORA_BANDWIDTH;
        config->modParams.Params.LoRa.CodingRate = LORA_CODINGRATE;
        config->modParams.Params.LoRa.LowDatarateOptimize = LORA_LOWDATARATEOPTIMIZE;
        config->modParams.Params.LoRa.SpreadingFactor = LORA_SPREADING_FACTOR;
        config->packetParams.PacketType = PACKET_TYPE_LORA;
        config->packetParams.Params.LoRa.CrcMode = LORA_CRC_MODE;
        config->packetParams.Params.LoRa.HeaderType = LORA_HEADER_TYPE;
        config->packetParams.Params.LoRa.InvertIQ = LORA_IQ;
        config->packetParams.Params.LoRa.PayloadLength = BUFFER_SIZE;
        config->packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    #elif USE_MODEM_FSK == 1
        config->packetType = PACKET_TYPE_GFSK;
        config->modParams.PacketType = PACKET_TYPE_GFSK;
        config->modParams.Params.Gfsk.Bandwidth = FSK_BANDWIDTH;
        config->modParams.Params.Gfsk.BitRate = 1024000000 / FSK_DATARATE;
        config->modParams.Params.Gfsk.Fdev = FSK_FDEV * 1.048576;
        config->modParams.Params.Gfsk.ModulationShaping = FSK_MODULATION_SHAPPING;
        config->packetParams.PacketType = PACKET_TYPE_GFSK;
        config->packetParams.Params.Gfsk.AddrComp = FSK_ADDR_FILTERING;
        config->packetParams.Params.Gfsk.CrcLength = FSK_CRC_MODE;
        config->packetParams.Params.Gfsk.DcFree = FSK_WHITENING_MODE;
        config->packetParams.Params.Gfsk.HeaderType = FSK_HEADER_TYPE;
        config->packetParams.Params.Gfsk.PayloadLength = BUFFER_SIZE;
        config->packetParams.Params.Gfsk.PreambleLength = FSK_PREAMBLE_LENGTH;
        config->packetParams.Params.Gfsk.PreambleMinDetect = FSK_PREAMBLE_DETECTOR_MODE;
        config->packetParams.Params.Gfsk.SyncWordLength = FSK_SYNCWORD_LENGTH;
    #endif
}

void sx126x_Reset() {
      //__disable_irq( );
      HAL_Delay(20);
      HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, 0);
      HAL_Delay(50);
      HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, 1);
      //RadioReset.input( ); // Using the internal pull-up
      HAL_Delay(20);
      //__enable_irq( );

}

void sx126x_SetStandby( RadioStandbyModes_t standbyConfig )
{
    sx126x_WriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
    if( standbyConfig == STDBY_RC )
    {
        OperatingMode = MODE_STDBY_RC;
    }
    else
    {
        OperatingMode = MODE_STDBY_XOSC;
    }
}

void sx126x_Init() {
  sx126x_Reset();
  sx126x_Wakeup();
  sx126x_SetStandby( STDBY_RC );

  //SetPollingMode( );

  sx126x_AntSwOn( );
  sx126x_SetDio2AsRfSwitchCtrl( true );


  OperatingMode = MODE_STDBY_RC;

  sx126x_SetPacketType( PACKET_TYPE_LORA );

  #ifdef USE_CONFIG_PUBLIC_NETOWRK
    // Change LoRa modem Sync Word for Public Networks
    sx126x_WriteReg( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF , 1);
    sx126x_WriteReg( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF , 1);
  #else
    // Change LoRa modem SyncWord for Private Networks
    sx126x_WriteReg( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF , 1);
    sx126x_WriteReg( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF , 1);
  #endif
}


void sx126x_WriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    sx126x_WaitOnBusy( );

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);
    uint8_t buf = (uint8_t)command;
    HAL_SPI_Transmit(&hspi1, &buf, 1, HAL_MAX_DELAY);
    //for( uint16_t i = 0; i < size; i++ )
    //{
        HAL_SPI_Transmit(&hspi1, buffer, size, HAL_MAX_DELAY);
    //}

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 1);


    sx126x_WaitOnCounter( );
}

void sx126x_WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    sx126x_WaitOnBusy( );

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);
    uint8_t buf = RADIO_WRITE_BUFFER;
    HAL_SPI_Transmit(&hspi1, &buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &offset, 1, HAL_MAX_DELAY);
    //for( uint16_t i = 0; i < size; i++ )
    //{
      HAL_SPI_Transmit(&hspi1, buffer, size, HAL_MAX_DELAY);
    //}
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);

}

void sx126x_WriteReg( uint16_t address, uint8_t *buffer, uint16_t size )
{
    sx126x_WaitOnBusy( );

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);
    uint8_t buf[3];

    buf[0] = ( uint8_t )RADIO_WRITE_REGISTER;
    buf[1] = ( uint8_t )( address & 0xFF00 ) >> 8;
    buf[2] = ( uint8_t )( address & 0x00FF );

    HAL_SPI_Transmit(&hspi1, buf, 3, HAL_MAX_DELAY);

    //for( uint16_t i = 0; i < size; i++ )
    //{
        HAL_SPI_Transmit(&hspi1, buffer, size, HAL_MAX_DELAY);
    //}
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 1);
}

void sx126x_WaitOnCounter( ) {
  for( uint8_t counter = 0; counter < 15; counter++ )
    {  __NOP( ); }
}

void sx126x_SetDio2AsRfSwitchCtrl( uint8_t enable ){
    sx126x_WriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}


void sx126x_Wakeup( void )
{
    //__disable_irq( );

    //Don't wait for BUSY here

    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);
    uint8_t buf = RADIO_GET_STATUS;
    HAL_SPI_Transmit(&hspi1, &buf, 1, HAL_MAX_DELAY);
    buf = 0;
    HAL_SPI_Transmit(&hspi1, &buf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 1);


    // Wait for chip to be ready.
    sx126x_WaitOnBusy( );

    //__enable_irq( );

    sx126x_AntSwOn( );
}

void sx126x_AntSwOn()
{
    HAL_GPIO_WritePin(ANTSW_GPIO_Port, ANTSW_Pin, 1);
}

void sx126x_AntSwOff()
{
    HAL_GPIO_WritePin(ANTSW_GPIO_Port, ANTSW_Pin, 0);
}

void sx126x_WaitOnBusy() {
  while (HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin) == 1) {}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|NSS_Pin|ANTSW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin ANTSW_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|ANTSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
