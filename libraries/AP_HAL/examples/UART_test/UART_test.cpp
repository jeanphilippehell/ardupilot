/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/AP_BoardLED.h>        // Board LED library
#include <hwdef.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_Common/AP_Common.h>
#include <AP_InternalError/AP_InternalError.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();

const char *_log_directory = "New_Logs";
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_BoardLED board_led;
AP_HAL::Storage *storage = hal.storage;
struct stat st;
int ret;
int write_fd;
char filename[30];
char *write_filename = & filename[0];
uint16_t log_num = 0;
uint32_t last_periodic_1Hz;
uint8_t toggle = 0;

/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
}


void setup(void)
{
    // initialise the board leds
    board_led.init();
    storage->init();

    hal.scheduler->delay(1000);

    setup_uart(hal.uartA, "uartA");  // console
    setup_uart(hal.uartB, "uartB");  // 1st GPS
    setup_uart(hal.uartC, "uartC");  // telemetry 1
    setup_uart(hal.uartD, "uartD");  // telemetry 2
    setup_uart(hal.uartE, "uartE");  // 2nd GPS

    storage->init();
    unsigned char buff[8], XOR_res = 0;

    for (uint32_t i = 0; i < HAL_STORAGE_SIZE; i += 8) {
        storage->read_block((void *) buff, i, 8);
        for(uint32_t j = 0; j < 8; j++) {
            XOR_res ^= buff[j];
        }
    }

    /*
      print XORed result
     */
    hal.console->printf("XORed ememory: %u\r\n", (unsigned) XOR_res);

    EXPECT_DELAY_MS(3000);
    ret = AP::FS().stat(_log_directory, &st);
    if (ret == -1) {
        ret = AP::FS().mkdir(_log_directory);
    }
    if (ret == -1 && errno != EEXIST) {
        printf("Failed to create log directory %s : %s\n", _log_directory, strerror(errno));
        _log_directory = "";
    }

    snprintf(filename, sizeof(filename), "%s/LOG%03u.BIN", _log_directory, (unsigned)log_num);

    EXPECT_DELAY_MS(3000);
    while (AP::FS().stat(write_filename, &st) != -1) {
        // hopefully errno==ENOENT.  If some error occurs it is
        // probably better to assume this file exists.
        log_num++;
        snprintf(filename, sizeof(filename), "%s/LOG%03u.BIN", _log_directory, (unsigned)log_num);
    }

    write_fd = AP::FS().open(write_filename, O_WRONLY|O_CREAT|O_EXCL|O_TRUNC);
    if (write_fd == -1) {
        printf("Log open fail for %s - %s\n", write_filename, strerror(errno));
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    }
    else {
        printf("Log %s openned\n", write_filename);
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
    }
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds\n",
                 name, (double)(AP_HAL::millis() * 0.001f));
    if (uart->available()) {

    }
}

void loop(void)
{
    if (hal.uartC->available()) {
        int16_t b = hal.uartC->read();
        //hal.uartA->write(b);
        if (write_fd != -1) {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
            AP::FS().write(write_fd, &b, 1);
            //AP::FS().close(write_fd);
        }
    }
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);

    uint32_t now = AP_HAL::millis();
    if (now - last_periodic_1Hz > 1000) {
        last_periodic_1Hz = now;
        if (write_fd != -1) {
            if (toggle == 0) {
                toggle ++;
                //hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
            }
            else {
                toggle = 0;
                AP::FS().fsync(write_fd);
                //hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            }
        }
    }
}

AP_HAL_MAIN();
