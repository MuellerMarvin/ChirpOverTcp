// Include Libraries
#include <driver/i2s.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include <mySD.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#pragma region Config

#define I2S_WS 22 // Left right clock
#define I2S_SD 21 // Serial data
#define I2S_SCK 25 // Serial clock

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (3) //Seconds
#define I2S_CHANNEL_NUM   (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)

const char WiFiName[] = "BALISONG";
const char WiFiPass[] = "7422jT3]";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

const int wavHeaderSize = 44;
char* filename = "rec.wav";
bool isRecording = false;
const char tcpHostIpV4[] = "192.168.0.164";
const long port = 13339;
#pragma endregion


#pragma region Setup

void setup() {
#pragma region Wifi
	Serial.println("Connecting to WiFi...");
	WiFi.begin(WiFiName, WiFiPass);
	while (!WiFi.isConnected()) {
		//no-op until connected
	}
	Serial.println("WiFi-connection established.");
#pragma endregion

#pragma region SD-card Init
	try
	{
		sdInit();
	}
	catch (const std::exception&)
	{
		Serial.println("SD-Card initialization failed.");
		return;
	}
#pragma endregion

#pragma region Microphone Init
	Serial.println("Initializing microphone...");
	try
	{
		i2sInit();
	}
	catch (const std::exception&)
	{
		Serial.println("Mirophone initialization failed.");
		return;
	}
	Serial.println("Microphone initialized.");
#pragma endregion

	//#pragma region ClockInit
	//// Initialize a NTPClient to get time
	//timeClient.begin();
	//// Set offset time in seconds to adjust for your timezone, for example:
	//// GMT +1 = 3600
	//// GMT +8 = 28800
	//// GMT -1 = -3600
	//// GMT 0 = 0
	//timeClient.setTimeOffset(3600);
	//#pragma endregion

	Serial.println("Setup complete");
}

#pragma endregion



void loop() {
	if (!isRecording) {
		isRecording = true; // the CPU would be too quick and record double if only set in the created thread
		xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL); // record asyncronously
	}
}

void sendFile() {
	File recFile = SD.open(filename, FILE_READ);

	WiFiClient client;

	if (!client.connect(tcpHostIpV4, port, 10000))
	{
		Serial.println("Connection to server failed.");
		recFile.close();
		return;
	}

	Serial.println("Sending...");

	byte buffer[255];
	while (recFile.available())
	{
		// read from sd card
		recFile.read(buffer, sizeof(buffer));
		// send to server
		client.write(buffer, sizeof(buffer));
		client.flush();
	}
	client.stop();
	Serial.println("File sent to server - deleting file.");
}

void sdInit() {
	Serial.print("Initializing SD card...");
	/* initialize SD library with Soft SPI pins, if using Hard SPI replace with this SD.begin()*/
	if (!SD.begin(5, 23, 19, 18)) {
		Serial.println("SD-Card initialization failed.");
		return;
	}
}

void i2sInit() {
	/*
	  Sets i2s config options
	  Installs driver
	  Sets pin options
	*/
	i2s_config_t i2s_config = {
	  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
	  .sample_rate = I2S_SAMPLE_RATE,
	  .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
	  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
	  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
	  .intr_alloc_flags = 0,
	  .dma_buf_count = 64,
	  .dma_buf_len = 1024,
	  .use_apll = 1
	};

	i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

	const i2s_pin_config_t pin_config = {
	  .bck_io_num = I2S_SCK,
	  .ws_io_num = I2S_WS,
	  .data_out_num = -1,
	  .data_in_num = I2S_SD
	};

	i2s_set_pin(I2S_PORT, &pin_config);
}

void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len) {
	/*
	  save original data from I2S(ADC) into flash
	*/
	uint32_t j = 0;
	uint32_t dac_value = 0;
	for (int i = 0; i < len; i += 2) {
		dac_value = ((((uint16_t)(s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
		d_buff[j++] = 0;
		d_buff[j++] = dac_value * 256 / 2048;
	}
}

void i2s_adc(void* arg) {
	// mark as recording
	isRecording = true;

	//timeClient.update();
	//unsigned long epoch = timeClient.getEpochTime();
	//File currentlyRecording = SD.open(timeClient.getFormattedTime().c_str(), FILE_WRITE);
	File currentlyRecording = SD.open(filename, FILE_WRITE);
	// write header
	byte header[wavHeaderSize];
	wavHeader(header, FLASH_RECORD_SIZE, wavHeaderSize);
	currentlyRecording.write(header, wavHeaderSize);

	int i2s_read_len = I2S_READ_LEN;
	int flash_wr_size = 0;
	size_t bytes_read;

	char* i2s_read_buff = (char*)calloc(i2s_read_len, sizeof(char));
	uint8_t* flash_write_buff = (uint8_t*)calloc(i2s_read_len, sizeof(char));

	i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);

	// Record
	Serial.println("Recording a new file.");
	while (flash_wr_size < FLASH_RECORD_SIZE) {
		//read data from I2S bus, in this case, from ADC.
		i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
		//save original data from I2S(ADC) into flash.
		i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
		currentlyRecording.write((const byte*)flash_write_buff, i2s_read_len);
		flash_wr_size += i2s_read_len;
	}

	// close file
	currentlyRecording.flush();
	currentlyRecording.close();

	Serial.println("Recording completed.");
	delay(100);

	// send file
	sendFile();

	free(i2s_read_buff);
	i2s_read_buff = NULL;
	free(flash_write_buff);
	flash_write_buff = NULL;
	// open up for new recordings before the task is deleted
	isRecording = false;
	vTaskDelete(NULL);
}

static void wavHeader(byte* header, int wavSize, int headerSize) {
	header[0] = 'R';
	header[1] = 'I';
	header[2] = 'F';
	header[3] = 'F';
	unsigned int fileSize = wavSize + headerSize - 8;
	header[4] = (byte)(fileSize & 0xFF);
	header[5] = (byte)((fileSize >> 8) & 0xFF);
	header[6] = (byte)((fileSize >> 16) & 0xFF);
	header[7] = (byte)((fileSize >> 24) & 0xFF);
	header[8] = 'W';
	header[9] = 'A';
	header[10] = 'V';
	header[11] = 'E';
	header[12] = 'f';
	header[13] = 'm';
	header[14] = 't';
	header[15] = ' ';
	header[16] = 0x10;
	header[17] = 0x00;
	header[18] = 0x00;
	header[19] = 0x00;
	header[20] = 0x01;
	header[21] = 0x00;
	header[22] = 0x01;
	header[23] = 0x00;
	header[24] = 0x80;
	header[25] = 0x3E;
	header[26] = 0x00;
	header[27] = 0x00;
	header[28] = 0x00;
	header[29] = 0x7D;
	header[30] = 0x00;
	header[31] = 0x00;
	header[32] = 0x02;
	header[33] = 0x00;
	header[34] = 0x10;
	header[35] = 0x00;
	header[36] = 'd';
	header[37] = 'a';
	header[38] = 't';
	header[39] = 'a';
	header[40] = (byte)(wavSize & 0xFF);
	header[41] = (byte)((wavSize >> 8) & 0xFF);
	header[42] = (byte)((wavSize >> 16) & 0xFF);
	header[43] = (byte)((wavSize >> 24) & 0xFF);
}
