<h1>GIỚI THIỆU</h1>

Cảm biên sGY-BM 280 là cảm biến đo được áp suất không khí, nhiệt độ, và từ sự thay đổi của áp 

suất không khí theo độ cao thì nó có thể tính thêm được 1 giá trị nữa đó là độ cao.

<h1>Interface</h1>

Cảm biến hỗ trợ 2 loại giao tiếp cơ bản là SPI và I2C.

SPI: Để giao tiếp SPI ta nối chân CSB với GND (up to 10 Mhz).

I2C: Để giao tiếp I2C ta nối chân CSB với VCC(up to 3.4 Mhz)

Lưu ý: 

SDO kết nối với GND thì Cảm biến là Slaver.

SDO kết nối VCC thì Cảm biến là Master.

<h2>I2C</h2>

    Cảm biến hỗ trợ 3 Mode sử dụng cho I2C đó là : 

    Standard

    Fast

    High Speed

    Để sử dụng I2C ta cần phải có I2CAddress với định danh là : 1111011X

    X = 0: SDO kết nối GND thì slave address là 11110110(0x76)

    X = 1: SDO kết nối VDD thì slave address là 11110111(0x77)

<h3>I2C_Write</h3>

    Để có thể write được thì đầu tiền sẽ phải gửi 1 slave address ở chế độ write ( RW = 0) Và 
    
    slave address sẽ có định dạng 1111011X0( X phụ thuộc vào SDO như bên trên). Sau đó Master sẽ 
    
    gửi các cặp thanh ghi địa chỉ và giữ liệu theo format ví dụ : 

<h3>I2C_Read</h3>

    Để có thể đọc được thanh ghi, đầu tiên địa chỉ thanh ghi phải được gửi đi trong chế độ write
    
    ( slave address 1110111X0) .  Sau đó 1 stop hoặc 1 điều kiện lặp lại phải được tạo ra. Tiếp 
    
    theo 1 slave address được chỉ định ở  chế độ đọc RW = 1 sẽ được gửi đi . Và slave sẽ gửi 
    
    output data từ thanh ghi auto-incremented cho đến khi gặp 1 bit NOACKM và dừng điều kiện 
    
    lại.

<h2>SPI</h2>

    SPI hỗ trợ 2 mode thông qua CPOL và CPHA với 2 mode 00 và 11.

    00 : xung nhịp của đồng hồ ở mức thấp (CPOL = 0) và dữ liệu được lấy mẫu khi chuyển từ thấp 
    
    sang cao (cạnh lên) (CPHA = 0).

    11 : xung nhịp của đồng hồ ở mức cao (CPOL = 1) và dữ liệu được lấy mẫu khi chuyển từ thấp 
    
    sang cao (cạnh xuông) (CPHA = 1).

    Và hỗ trợ 2 chế độ 3 dây và 4 dây (spi3w_en = 1 nếu sử dụng 3 dây).

    SPI chỉ sử dụng 7 bit của thanh ghi địa chỉ để sử dụng, MSB của thanh ghi địa chỉ không được
    
    sử dụng mà thay vào đó là bit read/write.

<h3>SPI_Write</h3>

    Chế độ write được thực hiện bằng cách hạ CSB và gửi các cặp byte điền khiển và thanh ghi dữ
    
    liệu. Byte điều khiển sẽ bao gồm SPI address ví dụ như hình bên dưới.

<h3>SPI_Read</h3>

    Chế độ Read được thực iện bằng cách hạ CSB xuống mức thấp và gửi 1 byte control đầu tiên. 
    
    Byte controls bao hồm địa cỉ SPI và bit read (bit số 7 RW = 1). Sau đó viết 1 byte control,
    
    dữ liệu sẽ được gửi ra bằng chân SDO. Địa chỉ thanh ghi sẽ tự động cộng thêm 1.

Tính toán giá trị nhiệt độ, áp suất và độ cao

    Giới hạn phạm vi tính toán

    Phạm vi áp suất đo : 300 – 1100 hPa

    Mức tương đối (700-900hPa, 25oC): (+-)0.12hPa tương đương với (+-)1m

    Mức tuyệt đối( 950-1050hPa, 0..40oC): (+-)1hPa

    Phạm vi nhiệt độ: -40…+85oC

<h2>Luồng tính toán giá trị</h2>

    Với các bài toán khác nhau thì ta có thể chọn skiped chức năng đo nhiệt độ, áp suất hoặc có 
    
    thể đo cả 2 giá trị này.

    Sau đó ta sẽ phải config chức năng IIR filter để có thể cho ra được giá trị chính xác nhất.

    Cuối cùng các giá trị thu được từ sensor sẽ được xử lý quá lớp filter và đi tới kết quả cuối 
    
    cùng để kết thúc 1 chu kỳ đo.

<h3>IIR là gì?</h3>

    Trong môi trường thực tế có rất nhiều tình huông slamf tay đổi áp suất tạm thời trong 1 
    
    khoảng thời gian ngắn như gió thổi, …. Bởi vậy ta cần có 1 bộ lọc để có thể lược bỏ bớt 
    
    những dữ liệu gây nhiễu như vậy để có được 1 kết quả đúng nhất. Công thức tính toán IIR được
    
    trình diễn như sau:

    Dữ liệu sẽ được tính theo công thức trên với : 

    data_filtered_old là dữ liệu đã lọc từ lần trước. 

    data_ADC là giá trị ADC lấy được trước khi lọc IIR.

    Giá trị IIR sẽ được config bằng 3 bit [2:0] tại thanh ghi 0xF5 và giá trị filter_coefficient
    
    sẽ được xác định thông qua bảng dưới đây:

<h3>Oversampling là gì?</h>

     Với các ứng dụng khác nhau ta cũng sẽ phải sử dụng các oversampling khác nhau để sao cho 
     
     phù hợp với yêu cầu bài toán. Có các mode oversampling sau:

     Ultra low power

     Low power

     Standard

     High resolution

     Ultra high resolution

     Các giá trị này sẽ được setting thông qua các tham số osrt_p và osrt_t và có giá trị tương
     
     đương như 2 bảng dưới đây

<h2>Tính toán kết quả</h2> 

Để đọc được giá trị của 3 đại lượng trên ta cần phải  lấy các giá trị từ thanh ghi cắt đã được

thiết kế sẵn, với nhiệt độ ta có 3 thanh ghi T1,T2,T3 và áp suất ta có 7 thanh ghi từ P1->P9 như

hình.

<h3>Nhiệt độ</h3>

BMP280_S32_t t_fine;

double bmp280_compensate_T_double(BMP280_S32_t adc_T)

    {
    double var1, var2, T;

    double var1, var2, T;

    var1 = (((double)adc_T)/16384.0 – ((double)dig_T1)/1024.0) * ((double)dig_T2);

    var2 = ((((double)adc_T)/131072.0 – ((double)dig_T1)/8192.0) * (((double)adc_T)/131072.0 – ((double) dig_T1)/8192.0)) * ((double)dig_T3);

    t_fine = (BMP280_S32_t)(var1 + var2);
    
    T = (var1 + var2) / 5120.0;
    
    return T;
    
    }

<h3>Áp Suất</h3>

double bmp280_compensate_P_double(BMP280_S32_t adc_P)

    {

    double var1, var2, p;
    
    var1 = ((double)t_fine/2.0) – 64000.0;
    
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    
    var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
    
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    
    var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);

    if (var1 == 0.0) {

    return 0; // avoid exception caused by division by zero

    }

    p = 1048576.0 – (double)adc_P;

    p = (p – (var2 / 4096.0)) * 6250.0 / var1;

    var1 = ((double)dig_P9) * p * p / 2147483648.0;
    
    var2 = p * ((double)dig_P8) / 32768.0;
    
    p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
    
    return p;
    }

<h3> Độ Cao</h3>

double bmp280_compensate_A_double(double seaLevel)
    {

    double atmospheric = bmp280_compensate_P_double(adc_P)/100.0F;

    return 44330.0 * (1.0 - pow(atmospheric /seaLevel, 0.1903);

    }

Tài Liệu Tham khảo

https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

https://github.com/adafruit/Adafruit_BME280_Library
