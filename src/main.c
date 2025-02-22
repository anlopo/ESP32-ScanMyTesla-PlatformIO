#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_ota_ops.h"
#include "bootloader_common.h"
#include <string.h>
#include <ctype.h>

#define BUFFER_LENGTH 16
#define RX_PIN 17
#define TX_PIN 16
#define BLINK_GPIO 2
#define LED_BLINK_INTERVAL 200
#define LED_OFF_TIME 500

#ifdef DEBUG_BUILD
static uint8_t debug_enabled_level = 1;
#else
static uint8_t debug_enabled_level = 0;
#endif

uint8_t messageCounter = 0;
bool ids[2048];
uint16_t canDataBufferId[BUFFER_LENGTH];
uint8_t canDataBufferLength[BUFFER_LENGTH];
uint8_t canDataBufferData[BUFFER_LENGTH][8];
bool noFilter = true, bt_echo = true;
uint32_t spp_connection_handle;
static bool led_state = false;
static bool led_blinking = false;

TaskHandle_t can_task_handle = NULL;
static TimerHandle_t led_blink_timer;
static TimerHandle_t led_off_timer;

#define CHUNK_SIZE 1024
#define HASH_SIZE 32
static size_t fw_size;
static esp_ota_handle_t ota_handle;
static const esp_partition_t *ota_partition;
static bool next_spp_data_is_size = false, next_spp_data_is_update = false;
static bool spp_send_ota_status_after_connect = false;
RTC_NOINIT_ATTR int ota_restart;

// Function that process CAN messages
static void process_can_message(twai_message_t *canMessage)
{
    // Retrieve the data length code from the CAN message
    uint8_t length = canMessage->data_length_code;

    // Check if the message identifier is not in the list of IDs we are interested in.
    // If not, exit the function early.
    if (!ids[canMessage->identifier])
        return;

    // Validate the data length. It must be between 1 and 8 bytes (inclusive).
    // If the length is out of this range, exit the function early.
    if (length == 0 || length > 8)
        return;

    // Calculate the line number in the buffer based on the message identifier.
    uint8_t lineNumber = canMessage->identifier % BUFFER_LENGTH;

    // Check if the current message matches the last received message for this slot.
    // If it does, there is no need to process it again, so exit the function early.
    if (canDataBufferId[lineNumber] == canMessage->identifier && canDataBufferData[lineNumber][0] == canMessage->data[0])
        return;

    // Update the buffer with the new message identifier
    canDataBufferId[lineNumber] = canMessage->identifier;

    // Set the length of data in this slot to the current message's length
    canDataBufferLength[lineNumber] = length;

    // Copy the message data into the appropriate slot in the buffer
    memcpy(canDataBufferData[lineNumber], canMessage->data, length);

    // Increment the message counter and wrap around if it reaches the buffer length
    messageCounter = (messageCounter + 1) % BUFFER_LENGTH;
}

// Initialize CAN
static bool can_init()
{
    // Configure the general settings for the CAN driver:
    //   TX_PIN: The GPIO pin used for transmitting data.
    //   RX_PIN: The GPIO pin used for receiving data.
    //   TWAI_MODE_LISTEN_ONLY: Sets the device to listen-only mode, meaning it will not send any frames.
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_LISTEN_ONLY);

    // Set up the timing configuration for the CAN driver at 500 kbits/s
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // Configure the filter to accept all incoming messages
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install the TWAI (CAN) driver with the provided configurations.
    // If installation fails, return false.
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
        return false;

    // Start the TWAI driver. If starting the driver fails, return false.
    if (twai_start() != ESP_OK)
        return false;

    // Reconfigure the alerts to monitor specific events:
    //   TWAI_ALERT_RX_DATA: Alert when new data is received.
    //   TWAI_ALERT_ERR_PASS: Alert on error passive state.
    //   TWAI_ALERT_BUS_ERROR: Alert on bus errors.
    //   TWAI_ALERT_RX_QUEUE_FULL: Alert when the receive queue is full.
    // Return true if configuring alerts succeeds, otherwise return false.
    return twai_reconfigure_alerts(TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL, NULL) == ESP_OK;
}

// Callback function for the LED blink timer.
// This function toggles the state of an LED connected to BLINK_GPIO.
static void led_blink_callback(TimerHandle_t xTimer)
{
    // Toggle the led_state variable between 0 and 1 (false and true).
    led_state = !led_state;

    // Set the GPIO pin level based on the new led_state.
    gpio_set_level(BLINK_GPIO, led_state);
}

// Callback function for the LED off timer.
// This function turns off the LED and stops the blink timer.
static void led_off_callback(TimerHandle_t xTimer)
{
    // Set the GPIO pin level to 0 (LOW), effectively turning off the LED.
    gpio_set_level(BLINK_GPIO, 0);

    // Stop the blink timer.
    xTimerStop(led_blink_timer, 0);

    // Update the flag indicating that the LED is no longer blinking.
    led_blinking = false;
}

// Function to start the LED blinking process.
static void start_led_blinking(void)
{
    // Check if the LED is not already blinking.
    if (!led_blinking)
    {
        // Start the blink timer.
        xTimerStart(led_blink_timer, 0);

        // Set the flag indicating that the LED is now blinking.
        led_blinking = true;
    }

    // Start the off timer to stop the blinking after a certain period.
    xTimerStart(led_off_timer, 0);
}

// Task to handle CAN messages
static void can_task(void *arg)
{
    // Infinite loop to continuously check for CAN alerts and process incoming messages
    while (1)
    {
        // Variable to store the alerts that have been triggered
        uint32_t alerts_triggered;

        // Check if any alerts have been triggered by reading the alerts with a timeout of 1 millisecond.
        // If ESP_OK is returned and the TWAI_ALERT_RX_DATA alert is set, indicating new data has been received,
        // proceed to process the incoming messages.
        if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1)) == ESP_OK && (alerts_triggered & TWAI_ALERT_RX_DATA))
        {
            // Create a structure to hold the received message
            twai_message_t message;

            // Continuously receive CAN messages as long as there are messages available in the buffer.
            // Process each received message by calling process_can_message.
            while (twai_receive(&message, 0) == ESP_OK)
            {
                process_can_message(&message);
                start_led_blinking();
            }
        }

        // Delay the task for 10 milliseconds before checking for new alerts again
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Function that trim incoming BT Serial messages
static void trim_whitespace(char *str)
{
    // Check if the input string is NULL or empty. If so, return immediately.
    if (str == NULL || *str == '\0')
        return;

    // Pointer to the start of the string, used to skip leading whitespace characters
    char *start = str;
    while (isspace((unsigned char)*start))
    {
        start++;
    }

    // If only whitespace characters were found and the new start is at the end of the string,
    // set the original string to be empty.
    if (*start == '\0')
    {
        *str = '\0';
        return;
    }

    // Pointer to the end of the string, used to trim trailing whitespace characters
    char *end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end))
    {
        *end = '\0'; // Overwrite the current character with a null terminator
        end--;       // Move the end pointer backward
    }

    // If there are leading whitespace characters to remove,
    // move the trimmed string content to the beginning of the buffer.
    if (start != str)
    {
        memmove(str, start, strlen(start) + 1);
    }
}

// Function that send messages back to BT Serial
static void send_bt_message(char *response)
{
    static const char *TAG = "send_bt_message()";

    // Check if there is an active SPP (Serial Port Profile) connection.
    // If the connection handle is 0, it means there is no active connection.
    if (spp_connection_handle == 0)
    {
        ESP_LOGE(TAG, "No active SPP connection");
        return;
    }

    // Attempt to send the response data over the SPP connection.
    // The function esp_spp_write writes data to the specified SPP connection handle.
    esp_err_t write_result = esp_spp_write(spp_connection_handle, strlen(response), (uint8_t *)response);

    // Check if the write operation was successful.
    if (write_result != ESP_OK)
    {
        // If writing failed, log an error message.
        ESP_LOGE(TAG, "Failed to send data via SPP");
    }
    else
    {
        // If writing succeeded, log a debug message with the sent response.
        ESP_LOGD(TAG, "Sent response: %s", response);
    }
}

// Write data from SPP to ota, then check it
static uint8_t writeOTAdata(uint8_t *data, int len)
{
    static const char *TAG = "writeOTAdata()";

    static int received = 0; // Keeps track of the total data received for OTA update.
    esp_err_t err;           // Used to store the result of ESP-IDF API calls.

    if (received == 0)
    {
        // Get the next available partition for updating firmware.
        ota_partition = esp_ota_get_next_update_partition(NULL);

        // Start the OTA update process on the selected partition with a specified size.
        err = esp_ota_begin(ota_partition, fw_size, &ota_handle);
        if (err != ESP_OK) // Check if the operation was successful.
        {
            ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
            return 1; // Exit function on failure.
        }
    }

    // Write a chunk of data to the OTA partition.
    err = esp_ota_write(ota_handle, data, len);
    if (err != ESP_OK) // Check for errors during writing.
    {
        ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
        return 1; // Exit function on failure.
    }

    received += len; // Update the total data written.

    // Check if all data has been received.
    if (received >= fw_size)
    {
        next_spp_data_is_update = false;

        err = esp_spp_disconnect(spp_connection_handle);
        if (err != ESP_OK) // Check for errors during hash calculation.
            ESP_LOGE(TAG, "Can't disconnect SPP");
        else
            ESP_LOGD(TAG, "SPP disconnected");

        // Finalize the OTA update process.
        err = esp_ota_end(ota_handle);
        if (err != ESP_OK) // Check for errors during finalization.
        {
            ESP_LOGE(TAG, "esp_ota_end(): %s", esp_err_to_name(err));
            return 1; // Exit function on failure.
        }

        // Get the description of the OTA partition to log metadata.
        esp_app_desc_t ota_desc;
        err = esp_ota_get_partition_description(ota_partition, &ota_desc);
        if (err != ESP_OK) // Check for errors during retrieval.
        {
            ESP_LOGE(TAG, "Can't get OTA partition description: %s", esp_err_to_name(err));
            return 1; // Exit function on failure.
        }
        else
        {
            // Log various metadata about the updated firmware.
            ESP_LOGI(TAG, "OTA version='%s'", ota_desc.version);
            ESP_LOGI(TAG, "OTA name='%s'", ota_desc.project_name);
            ESP_LOGI(TAG, "OTA build time='%s'", ota_desc.time);
            ESP_LOGI(TAG, "OTA build date='%s'", ota_desc.date);
            ESP_LOGI(TAG, "OTA idf version='%s'", ota_desc.idf_ver);
            ESP_LOGI(TAG, "OTA part type='%u'", ota_partition->type);
            ESP_LOGI(TAG, "OTA part subtype='%u'", ota_partition->subtype);
            ESP_LOGI(TAG, "OTA part size='0x%08x'", (unsigned int)ota_partition->size);
            ESP_LOGI(TAG, "OTA part address='0x%08x'", (unsigned int)ota_partition->address);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, ota_desc.app_elf_sha256, HASH_SIZE, ESP_LOG_INFO);
        }

        // Calculate the SHA-256 hash of the OTA partition using bootloader API.
        uint8_t part_hash_metadata[HASH_SIZE], part_hash[HASH_SIZE];
        err = bootloader_common_get_sha256_of_partition(ota_partition->address, fw_size, ota_partition->type, part_hash_metadata);
        if (err != ESP_OK) // Check for errors during hash calculation.
        {
            ESP_LOGE(TAG, "Can't get OTA partition hash: %s", esp_err_to_name(err));
            return 1; // Exit function on failure.
        }
        else
        {
            ESP_LOG_BUFFER_HEX_LEVEL("bootloader_common_get_sha256()", part_hash_metadata, HASH_SIZE, ESP_LOG_INFO);
        }

        // Calculate the SHA-256 hash of the OTA partition using ESP-IDF API.
        err = esp_partition_get_sha256(ota_partition, part_hash);
        if (err != ESP_OK) // Check for errors during hash calculation.
        {
            ESP_LOGE(TAG, "Can't get OTA APP hash: %s", esp_err_to_name(err));
            return 1; // Exit function on failure.
        }
        else
        {
            ESP_LOG_BUFFER_HEX_LEVEL("esp_partition_get_sha256()", part_hash_metadata, HASH_SIZE, ESP_LOG_INFO);
        }

        // Compare the two hashes to verify integrity of the firmware update.
        if (memcmp(part_hash_metadata, part_hash, HASH_SIZE) != 0)
        {
            ESP_LOGE(TAG, "Firmware hash mismatch");
            return 1; // Exit function on failure.
        }

        // Set the boot partition to the newly updated partition.
        esp_ota_set_boot_partition(ota_partition);

        // Log success and restart device after a short delay.
        ESP_LOGI(TAG, "Update successful, restarting...");
        ota_restart = 1;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second.
        esp_restart();                   // Restart the system to boot the new firmware.
        return 2;
    }
    return 0;
}

// Function that handle incoming BT Serial messages
static void process_bt_message(char *bt_message)
{
    static const char *TAG = "process_bt_message()";

    // Send the received message back (echo).
    if (bt_echo)
    {
        send_bt_message(bt_message);
        send_bt_message("\r");
    }

    // Check if the next expected data is the firmware size.
    if (next_spp_data_is_size)
    {
        next_spp_data_is_size = false;          // Reset flag after processing the size.
        fw_size = strtol(bt_message, NULL, 10); // Convert received message to integer for firmware size.

        // Validate the firmware size: should be greater than 0 and less than 2MB.
        if (fw_size > 0 && fw_size < 2 * 1024 * 1000)
        {
            next_spp_data_is_update = true;                   // Set flag to expect firmware data next.
            send_bt_message("OK SEND FW in RAW BINARY\r\n>"); // Inform sender to start sending firmware data.
        }
        else
        {
            send_bt_message("ERR INVALID SIZE\r\n>"); // Notify sender of invalid size.
        }
        return; // Exit function after handling the size message.
    }

    // Check if the received message is "ATMA" or "STM" to send CAN data buffer contents.
    if (strcasecmp(bt_message, "ATMA") == 0 || strcasecmp(bt_message, "STM") == 0)
    {
        char returnToSmt[256] = {0}; // Buffer to hold the response message.

        for (uint8_t i = 0; i < BUFFER_LENGTH; i++)
        {
            if (canDataBufferId[i] == 0) // Skip empty buffer entries.
                continue;

            char tempBuffer[32];

            // Append the CAN ID in hexadecimal format.
            snprintf(tempBuffer, sizeof(tempBuffer), "%03X", canDataBufferId[i]);
            strncat(returnToSmt, tempBuffer, sizeof(returnToSmt) - strlen(returnToSmt) - 1);

            // Append the CAN data bytes in hexadecimal format.
            for (uint8_t l = 0; l < canDataBufferLength[i]; l++)
            {
                snprintf(tempBuffer, sizeof(tempBuffer), "%02X", canDataBufferData[i][l]);
                strncat(returnToSmt, tempBuffer, sizeof(returnToSmt) - strlen(returnToSmt) - 1);
            }

            // Mark the buffer entry as processed.
            canDataBufferId[i] = 0;
            strncat(returnToSmt, "\n", sizeof(returnToSmt) - strlen(returnToSmt) - 1); // Append newline character.
        }

        // Handle empty buffer
        if (returnToSmt[0] == '\0')
        {
            ESP_LOGI(TAG, "Nothing to return");
        }
        else
        {
            send_bt_message(returnToSmt);
        }
        send_bt_message("\r\n");
    }
    // Check if the received message is "DBG2" to enable debugging.
    else if (strcasecmp(bt_message, "DBG2") == 0)
    {
        debug_enabled_level = 2;
        esp_log_level_set("*", ESP_LOG_DEBUG); // Set log level to DEBUG.
        send_bt_message("OK\r\n");
        ESP_LOGI(TAG, "Debugging enabled");
    }
    // Check if the received message is "DBG1" to enable debugging.
    else if (strcasecmp(bt_message, "DBG1") == 0)
    {
        debug_enabled_level = 1;
        esp_log_level_set("*", ESP_LOG_WARN); // Set log level to DEBUG.
        send_bt_message("OK\r\n");
        ESP_LOGI(TAG, "Debugging enabled (WARN)");
    }
    // Check if the received message is "DBG0" to disable debugging.
    else if (strcasecmp(bt_message, "DBG0") == 0)
    {
        debug_enabled_level = 0;
        ESP_LOGI(TAG, "Debugging disabled");
        esp_log_level_set("*", ESP_LOG_ERROR); // Set log level to ERROR.
        send_bt_message("OK\r\n");
    }
    else if (strcasecmp(bt_message, "ATE0") == 0)
    {
        bt_echo = false;
        ESP_LOGI(TAG, "Echo off");
        send_bt_message("OK\r\n");
    }
    else if (strcasecmp(bt_message, "ATE1") == 0)
    {
        bt_echo = true;
        ESP_LOGI(TAG, "Echo on");
        send_bt_message("OK\r\n");
    }
    else if (strncasecmp(bt_message, "ATSP", 4) == 0)
    {
        uint8_t protocol = strtol(bt_message + 4, NULL, 10);
        if (protocol == 6 || protocol == 0)
        {
            ESP_LOGI(TAG, "Chosen CAN protocol");
            send_bt_message("OK\r\n");
        }
        else
        {
            ESP_LOGW(TAG, "Chosen protocol: %d", protocol);
            send_bt_message("OK\r\n");
        }
    }
    else if (strncasecmp(bt_message, "STP", 3) == 0)
    {
        uint8_t protocol = strtol(bt_message + 3, NULL, 10);
        if (protocol == 31 || protocol == 33)
        {
            ESP_LOGI(TAG, "Chosen CAN protocol");
            send_bt_message("OK\r\n");
        }
        else
        {
            ESP_LOGW(TAG, "Chosen protocol: %d", protocol);
            send_bt_message("OK\r\n");
        }
    }
    else if (strcasecmp(bt_message, "STMA") == 0)
    {
        ESP_LOGI(TAG, "Requested Monitor all messages on OBD bus.");
        send_bt_message("OK\r\n");
    }
    else if (strcasecmp(bt_message, "ATDP") == 0)
    {
        ESP_LOGI(TAG, "Requested protocol info");
        send_bt_message("ISO 15765-4 (CAN 11/500)\r\n");
    }
    else if (strcasecmp(bt_message, "STPRS") == 0)
    {
        ESP_LOGI(TAG, "Requested protocol info");
        send_bt_message("ISO 11898, 11-bit Tx, 500kbps, var DLC\r\n");
    }
    else if (strcasecmp(bt_message, "ATDPN") == 0)
    {
        ESP_LOGI(TAG, "Requested protocol number");
        send_bt_message("6\r\n");
    }
    else if (strcasecmp(bt_message, "STPR") == 0)
    {
        ESP_LOGI(TAG, "Requested protocol number");
        send_bt_message("31\r\n");
    }
    else if (strcasecmp(bt_message, "ATH1") == 0)
    {
        ESP_LOGI(TAG, "Requested headers ON");
        send_bt_message("OK\r\n");
    }
    else if (strcasecmp(bt_message, "ATS0") == 0)
    {
        ESP_LOGI(TAG, "Requested spaces OFF");
        send_bt_message("OK\r\n");
    }
    else if (strcasecmp(bt_message, "ATCAF0") == 0)
    {
        ESP_LOGI(TAG, "Requested CAN formating OFF");
        send_bt_message("OK\r\n");
    }
    // Check if the received message starts with "STFAP" to add a CAN filter.
    else if (strncasecmp(bt_message, "STFAP ", 6) == 0)
    {
        uint16_t filter = strtol(bt_message + 6, NULL, 16); // Extract the filter ID from the message.

        if (noFilter)
        {
            memset(ids, false, sizeof(ids)); // Initialize all filters to false.
            noFilter = false;
        }

        if (filter < 2048) // Check if the filter ID is within valid range.
        {
            ids[filter] = true; // Enable the specified filter.
            send_bt_message("OK\r\n");
            ESP_LOGI(TAG, "Added filter: %03X", filter);
        }
        else
        {
            send_bt_message("ERR\r\n");
            ESP_LOGW(TAG, "Invalid filter ID"); // Log an error if the filter ID is invalid.
        }
    }
    // Check if the received message is "STFCP" to clear all CAN filters.
    else if (strcasecmp(bt_message, "STFCP") == 0)
    {
        memset(ids, true, sizeof(ids)); // Initialize all filters to true (no filtering).
        noFilter = true;
        send_bt_message("OK\r\n");
        ESP_LOGI(TAG, "Cleared all filters");
    }
    else if (strcasecmp(bt_message, "ATZ") == 0)
    {
        ESP_LOGI(TAG, "Requested restart");
        send_bt_message("OK\r\n");
    }
    else if (strcasecmp(bt_message, "ATZ1") == 0)
    {
        send_bt_message("OK RESTART IN 1s\r\n");
        ESP_LOGI(TAG, "REBOOT in 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_err_t err = esp_spp_disconnect(spp_connection_handle);
        if (err != ESP_OK) // Check for errors during hash calculation.
            ESP_LOGE(TAG, "Can't disconnect SPP");
        else
            ESP_LOGD(TAG, "SPP disconnected");
        esp_restart();
    }
    else if (strcasecmp(bt_message, "ATI") == 0 || strcasecmp(bt_message, "AT@1") == 0)
    {
        send_bt_message("OK\r\n");
        const esp_partition_t *partition_running;
        esp_app_desc_t running_app;
        partition_running = esp_ota_get_running_partition();
        if (partition_running == NULL)
            ESP_LOGE(TAG, "Can't get running partition");
        esp_err_t err = esp_ota_get_partition_description(partition_running, &running_app);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Can't get partition description: %s", esp_err_to_name(err));
        else
        {
            ESP_LOGI(TAG, "OTA version='%s'", running_app.version);
            ESP_LOGI(TAG, "OTA name='%s'", running_app.project_name);
            ESP_LOGI(TAG, "OTA build time='%s'", running_app.time);
            ESP_LOGI(TAG, "OTA build date='%s'", running_app.date);
            ESP_LOGI(TAG, "OTA idf version='%s'", running_app.idf_ver);
            ESP_LOGI(TAG, "OTA part type='%u'", partition_running->type);
            ESP_LOGI(TAG, "OTA part subtype='%u'", partition_running->subtype);
            ESP_LOGI(TAG, "OTA part size='0x%08x'", (unsigned int)partition_running->size);
            ESP_LOGI(TAG, "OTA part address='0x%08x'", (unsigned int)partition_running->address);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, running_app.app_elf_sha256, 32, ESP_LOG_INFO);

            char response[105]; // 32 + 32 + 16 + 16 + 4 + 4 + 1
            char ota_name[5];
            if (partition_running->subtype == 0x10)
                snprintf(ota_name, sizeof(ota_name), "ota0");
            else
                snprintf(ota_name, sizeof(ota_name), "ota1");
            snprintf(response, sizeof(response), "%s %s %s %s (%s)\n",
                     running_app.project_name, running_app.version, running_app.date, running_app.time, ota_name);
            send_bt_message(response);
        }
    }
    else if (strcasecmp(bt_message, "STDI") == 0)
    {
        const esp_partition_t *partition_running;
        esp_app_desc_t running_app;
        partition_running = esp_ota_get_running_partition();
        if (partition_running == NULL)
            ESP_LOGE(TAG, "Can't get running partition");
        esp_err_t err = esp_ota_get_partition_description(partition_running, &running_app);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Can't get partition description: %s", esp_err_to_name(err));
        else
        {
            ESP_LOGI(TAG, "OTA version='%s'", running_app.version);
            ESP_LOGI(TAG, "OTA name='%s'", running_app.project_name);

            char response[70]; // 32 + 32 + 4 + 1
            snprintf(response, sizeof(response), "%s r%s\n", running_app.project_name, running_app.version);
            send_bt_message(response);
        }
    }
    // Check if the received message is "FWUPDATE" to initiate firmware update process.
    else if (strcasecmp(bt_message, "FWUPDATE") == 0)
    {
        send_bt_message("OK SEND FW SIZE in BYTES\r\n"); // Inform sender to send firmware size next.
        ESP_LOGW(TAG, "Going to FW Update Mode");        // Log that the device is entering firmware update mode.
        next_spp_data_is_size = true;                    // Set flag to expect firmware size next.
    }
    else
    {
        // If the message is not recognized, send a "?" and log the received message.
        send_bt_message("?\r\n");
        ESP_LOGW(TAG, "Received unknown command: '%s'", bt_message);
    }

    send_bt_message(">"); // Send prompt character indicating readiness for next command.
}

// Bluetooth SPP callback function
static void bt_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    static const char *TAG = "bt_spp_callback()";

    switch (event)
    {
    case ESP_SPP_SRV_OPEN_EVT: // This event is triggered when a server-mode Bluetooth SPP connection is successfully opened.
    case ESP_SPP_OPEN_EVT:     // This event is triggered when a client-mode Bluetooth SPP connection is successfully opened.
        ESP_LOGI(TAG, "SPP connection opened, handle=%lu", param->open.handle);
        spp_connection_handle = param->open.handle; // Store the connection handle for future reference.

        // If a flag indicates that OTA status should be sent after connecting, send it and then reset the flag.
        if (spp_send_ota_status_after_connect)
        {
            spp_send_ota_status_after_connect = false;
            send_bt_message("\n\nLast OTA OK\r\n");
        }
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP connection closed, resetting handle.");
        spp_connection_handle = 0; // Reset the connection handle when the connection is closed.
        break;

    case ESP_SPP_DATA_IND_EVT: // This event is triggered when data is received on an SPP channel.
    {
        if (spp_connection_handle == 0)
        {
            spp_connection_handle = param->data_ind.handle;
            ESP_LOGW(TAG, "Manually setting spp_connection_handle = %lu", spp_connection_handle);
        }

        // Check if the next data expected is an OTA update. If so, write the received data to the OTA function.
        if (next_spp_data_is_update)
        {
            // Stop can_task
            if (can_task_handle != NULL)
            {
                vTaskDelete(can_task_handle);
                can_task_handle = NULL;
            }

            // Write the data received to the OTA handler.
            if (writeOTAdata(param->data_ind.data, param->data_ind.len) == 1)
            {
                ESP_LOGE(TAG, "OTA FAILED, RESTART IN 1s");
                vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second.
                esp_restart();                   // Restart the system.
            }
            break;
        }

        char received_data[128] = {0};
        size_t copy_len = param->data_ind.len < sizeof(received_data) - 1 ? param->data_ind.len : sizeof(received_data) - 1;

        memcpy(received_data, param->data_ind.data, copy_len); // Copy data from the parameter buffer to a local buffer.
        received_data[copy_len] = '\0';                        // Null-terminate the string.

        trim_whitespace(received_data); // Remove leading and trailing whitespace from the received data.

        ESP_LOGI(TAG, "Received data: '%s', Handle: %lu", received_data, param->data_ind.handle);
        process_bt_message(received_data); // Process the received Bluetooth SPP message.
        break;
    }

    case ESP_SPP_WRITE_EVT:
        break;

    default:
        ESP_LOGW(TAG, "Unhandled event: %d", event);
        break;
    }
}

// Initialize Bluetooth module
void bt_init()
{
    static const char *TAG = "bt_init()";

    esp_err_t ret; // Variable to store the return value of functions

    // Initialize the Bluetooth controller with default configuration
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    // Enable the Bluetooth controller in classic Bluetooth mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BT controller enable failed: %d", ret); // Log error if enabling fails
        return;
    }

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid init failed: %d", ret); // Log error if initialization fails
        return;
    }

    // Enable Bluedroid stack
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid enable failed: %d", ret); // Log error if enabling fails
        return;
    }

    // Set the device name for Bluetooth
    ret = esp_bt_gap_set_device_name("ESP32_CAN_Sniffer");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set device name: %d", ret); // Log error if setting device name fails
        return;
    }

    // Set the scan mode of the Bluetooth device to connectable and general discoverable
    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set scan mode: %d", ret); // Log error if setting scan mode fails
        return;
    }

    // Register the callback function for SPP events
    ret = esp_spp_register_callback(bt_spp_callback);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPP register callback failed: %d", ret); // Log error if registration fails
        return;
    }

    // Configure SPP mode and other settings
    esp_spp_cfg_t bt_spp_config = {
        .mode = ESP_SPP_MODE_CB,    // Use callback-based communication
        .enable_l2cap_ertm = false, // Disable L2CAP ERTM (Enhanced Retransmission Mode)
        .tx_buffer_size = 0,        // Use default buffer size for transmission
    };

    // Initialize the SPP module with the configured settings
    ret = esp_spp_enhanced_init(&bt_spp_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPP init failed: %d", ret); // Log error if initialization fails
        return;
    }

    // Start an SPP server with the specified security level and role
    ret = esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, "ESP32_SPP_SERVER");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start SPP server: %d", ret); // Log error if starting server fails
        return;
    }
}

// COnfigure GPIO
void config_gpio()
{
    gpio_reset_pin(BLINK_GPIO);
    // Set the GPIO as a push/pull output
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

// Main app function
void app_main()
{
    static const char *TAG = "app_main()";

    // Set the log level based on the debug_enabled flag
    if (debug_enabled_level == 0)
        esp_log_level_set("*", ESP_LOG_ERROR); // Set log level to ERROR if debugging is not enabled
    else
        esp_log_level_set("*", ESP_LOG_WARN); // Set log level to DEBUG if debugging is enabled

    if (ota_restart == 1)
    {
        ESP_LOGI(TAG, "OTA OK");                  // Log that the OTA process is successful
        spp_send_ota_status_after_connect = true; // Set flag to send OTA status after connection
    }
    ota_restart = 0; // Reset the ota_restart flag

    // Initialize NVS flash storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // If there are no free pages or a new version of NVS is found,
        // erase all data and reinitialize NVS flash storage
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret); // Check for errors during NVS initialization

    // Log that the CAN initialization process is starting
    ESP_LOGI(TAG, "Initializing CAN...");

    // Attempt to initialize CAN module repeatedly until successful
    while (!can_init())
    {
        ESP_LOGI(TAG, "Retrying CAN init in 1s..."); // Log retry message
        vTaskDelay(pdMS_TO_TICKS(1000));             // Delay for 1 second before retrying
    }

    // Initialize all filters to true (no filtering).
    memset(ids, true, sizeof(ids));

    // Initialize Bluetooth module
    bt_init();

    // Initialize GPIO
    config_gpio();

    led_blink_timer = xTimerCreate("LED Blink Timer", pdMS_TO_TICKS(LED_BLINK_INTERVAL), pdTRUE, NULL, led_blink_callback);
    led_off_timer = xTimerCreate("LED Off Timer", pdMS_TO_TICKS(LED_OFF_TIME), pdFALSE, NULL, led_off_callback);

    // Create a FreeRTOS task for CAN operations
    xTaskCreate(can_task, "can_task", 4096, NULL, 5, &can_task_handle);
}