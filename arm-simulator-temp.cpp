/* Payload formatting

function Decoder(bytes, port) {
  var temperature = ((bytes[0] << 8) | bytes[1]) / 100;

  return {
    temperature: temperature,
  }
}
*/


#include "mbed.h"
#include "mbed_trace.h"
#include "mbed_events.h"
#include "LoRaWANInterface.h"
#include "Sht31.h"
#include "SX1276_LoRaRadio.h"

//define interval between sensor readings in milliseconds
#define TX_INTERVAL         10000

//define the minimal difference in temperature to construct a message 100 = 1 degree
#define TEMP_DELTA           200

//define the buffer size in number of bytes, 2 for temperature
#define BUFFER_SIZE 2

// Device credentials, register device as OTAA in The Things Network and copy credentials here
static uint8_t DEV_EUI[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t APP_EUI[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t APP_KEY[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// The port we're sending and receiving on
#define MBED_CONF_LORA_APP_PORT     15

// Peripherals (LoRa radio, temperature sensor and button)
SX1276_LoRaRadio radio(D11, D12, D13, D10, A0, D2, D3, D4, D5, D8, D9, NC, NC, NC, NC, A4, NC, NC);
Sht31 sht31(I2C_SDA, I2C_SCL);

// EventQueue is required to dispatch events around
static EventQueue ev_queue;

// Constructing Mbed LoRaWANInterface and passing it down the radio object.
static LoRaWANInterface lorawan(radio);

// Application specific callbacks
static lorawan_app_callbacks_t callbacks;

// LoRaWAN stack event handler
static void lora_event_handler(lorawan_event_t event);

//every interval, the temperature is checked, only when the difference is larger than the defined value, a message is sent
static void check_sensor(){
    uint8_t tx_buffer[BUFFER_SIZE] = { 0 };
    static int stored_temperature=0; // latest temperature sent over LoRaWAN
    float temp=sht31.readTemperature();
    uint16_t temperature = temp * 100; // make an int to reduce the payload size

    if(temperature>(stored_temperature+TEMP_DELTA) || temperature < (stored_temperature-TEMP_DELTA)){
        tx_buffer[0]=temperature>>8;
        tx_buffer[1]=temperature;

        printf("Sending %d bytes\r\n", BUFFER_SIZE);

        //send message over LoRaWAN
        int16_t retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, BUFFER_SIZE, MSG_UNCONFIRMED_FLAG);

        stored_temperature = temperature;

            // for some reason send() returns -1... I cannot find out why, the stack returns the right number. I feel that this is some weird Emscripten quirk
        if (retcode < 0) {
            retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - duty cycle violation\n")
                    : printf("send() - Error code %d\n", retcode);
            return;
        }
        printf("%d bytes scheduled for transmission\n", retcode);
    }
    ev_queue.call_in(TX_INTERVAL, check_sensor);
}


int main() {
    if (DEV_EUI[0] == 0x0 && DEV_EUI[1] == 0x0 && DEV_EUI[2] == 0x0 && DEV_EUI[3] == 0x0 && DEV_EUI[4] == 0x0 && DEV_EUI[5] == 0x0 && DEV_EUI[6] == 0x0 && DEV_EUI[7] == 0x0) {
        printf("Set your LoRaWAN credentials first!\n");
        return -1;
    }

    // Enable trace output for this demo, so we can see what the LoRaWAN stack does
    mbed_trace_init();

    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("LoRa initialization failed!\n");
        return -1;
    }

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Disable adaptive data rating
    if (lorawan.disable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("disable_adaptive_datarate failed!\n");
        return -1;
    }

    lorawan.set_datarate(5); // SF7BW125

    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;

    lorawan_status_t retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("Connection error, code = %d\n", retcode);
        return -1;
    }

    printf("Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

// This is called from RX_DONE, so whenever a message came in
static void receive_message()
{
    uint8_t rx_buffer[50] = { 0 };
    int16_t retcode;
    retcode = lorawan.receive(MBED_CONF_LORA_APP_PORT, rx_buffer,
                              sizeof(rx_buffer),
                              MSG_CONFIRMED_FLAG|MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        printf("receive() - Error code %d\n", retcode);
        return;
    }

    printf("Data received on port %d (length %d): ", MBED_CONF_LORA_APP_PORT, retcode);

    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\n");
}

// Event handler
static void lora_event_handler(lorawan_event_t event) {
    switch (event) {
        case CONNECTED:
            printf("Connection - Successful\n");
            ev_queue.call_in(TX_INTERVAL, check_sensor);
            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("Disconnected Successfully\n");
            break;
        case TX_DONE:
            printf("Message Sent to Network Server\n");
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("Transmission Error - EventCode = %d\n", event);
            break;
        case RX_DONE:
            printf("Received message from Network Server\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("Error in reception - Code = %d\n", event);
            break;
        case JOIN_FAILURE:
            printf("OTAA Failed - Check Keys\n");
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}
