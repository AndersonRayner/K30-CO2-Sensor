
struct K30_CO2_rx_struct 
{
    uint8_t header1;
    uint16_t CO2_value;
    uint8_t checksum;
}__attribute__((packed));

const static uint8_t K30_CO2_RX_STREAM_BUFFER_SIZE = sizeof(K30_CO2_rx_struct);

union K30_CO2_rx_data
{
    K30_CO2_rx_struct msg;
    uint8_t msgData[K30_CO2_RX_STREAM_BUFFER_SIZE];
};
