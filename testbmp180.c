// Chương trình mẫu để đọc dữ liệu từ cảm biến bmp180
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>

#define BMP180_IOCTL_READ_TEMPERATURE _IOR('b', 1, int)
#define BMP180_IOCTL_READ_PRESSURE    _IOR('b', 2, int)

// Tính độ cao tương đối từ áp suất
double calculate_altitude(long pressure) {
    const double p0 = 101325.0; // Áp suất mực nước biển
    double altitude = 44330.0 * (1.0 - pow((double)pressure / p0, 0.1903));
    return altitude * 100.0; // đơn vị cm
}

int main() {
    int fd = open("/dev/bmp180", O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }
    long temp, press;
    while (1) {
        // Đọc nhiệt độ
        if (ioctl(fd, BMP180_IOCTL_READ_TEMPERATURE, &temp) == -1) {
            perror("Failed to read temperature");
            break;
        }
        // Đọc áp suất
        if (ioctl(fd, BMP180_IOCTL_READ_PRESSURE, &press) == -1) {
            perror("Failed to read pressure");
            break;
        }
        // Độ cao tương đối
        double altitude = calculate_altitude(press);
        printf("Temperature: %.1f (°C) | Pressure: %.1f (hPa) | Altitude: %.2f (m)\n",temp / 10.0f, press / 100.0f, altitude / 100.0f);
        sleep(1); 
    }
    close(fd);
    return 0;
}
