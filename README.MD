Họ và tên thành viên nhóm:
    1.Trần Quốc Tuấn   MSSV:22146445    
    2.Trần Quốc Thạnh  MSSV:22146394
    3.Đặng Thành Tài   MSSV:22146398
    4.Võ Minh Tấn      MSSV:22146402

I. Giới thiệu 

    Dự án này hỗ trợ giao tiếp cảm biến áp suất BMP180 thông qua driver kernel trên Raspberry PiZero2W, dữ liệu nhận được từ BMP180 bao gồm nhiệt độ, áp suất và độ cao tương đối.
    
II. Yêu cầu phần cứng, phần mềm

    Raspberry Pi Zero 2 W, BMP180 kết nối qua I2C, các gói cài đặt cần thiết,...
    +Thêm thiết bị vào Device Tree bằng cách biên dịch file bcm2710-rpi-zero-2-w.dts :
        1.Copy file bcm2710-rpi-zero-2-w.dts vào thư mục boot của raspberryPi
        2.Tại thư mục boot nhập vào terminal:
        dtc -I dts -O dtb -o bcm2710-rpi-zero-2-w.dtb bcm2710-rpi-zero-2-w.dts

III. Biên dịch và cài đặt

    Bước 1: Trên terminal(hoặc các trình sell khác hỗ trợ) nhập make (Yêu cầu kết nối với raspberrypi, các file cùng thư mục, sẽ sinh ra file bmp180_driver.ko)

    Bước 2: Nạp module bằng cách nhập:
    sudo insmod bmp180_driver.ko
    (sau khi thành công có thể kiểm tra bằng nhiều cách nhập: dmesg để xem trạng thái ở lớp kernel sapce hoặc: ls /dev/ để xem có file tên bmp180 hay chưa,...) 

    Bước 3: Biên dịch file testbmp180.c bằng cách nhập:
    gcc -o run testbmp180.c -lm 
    (thư viện math để tính độ cao bên ngoài user-space cho thuận tiện)

    Bước 4: chạy bằng cách nhập:
    sudo ./run
    Kết quả hiển thị ví dụ: Temperature: 30.0 C | Pressure: 1007.0 hPa | Altitude: 52.17 m

IV. Ghi chú kỹ thuật

    Đơn vị nhiệt độ: 0.1°C (chia cho 10 để in ra)

    Đơn vị áp suất: Pa (chia cho 100 để in ra hPa)

    Đơn vị độ cao: m hoặc cm (tùy cách xử lý trong code)

    Công thức và thông tin có thể tra datasheet của BMP180 tại:https://www.alldatasheet.com/datasheet-pdf/pdf/1132068/BOSCH/BMP180.html








