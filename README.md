# توضیحات استاندارد ارتباط با رابط کاربری
برای پیاده‌سازی استاندارد از توابع داخل فایل fetch_functions.py استفاده می‌کنیم
## یک نمونه از پیاده‌سازی:
```python

def fetch_raw_imu()->mavlink.MAVLink_raw_imu_message:
#we use the get functions written by the programmer here:
  compass_data=get_compass_data
  gyro_data=get_gyro_data
  accel_data=get_accel_data



    return mavlink.MAVLink_raw_imu_message(
        time_usec=time_usec_handler(),
        xacc=accel_data["x"],
        yacc=accel_data["y"],
        zacc=accel_data["z"],
        xgyro=gyro_data["x"],
        ygyro=gyro_data["y"],
        zgyro=gyro_data["z"],
        xmag=compass_data["x"],
        ymag=compass_data["y"],
        zmag=compass_data["z"],
        id=65535,
        temperature=65535

    )


```
ساختار توابع fetch به این صورت است که یک تابع را برمی‌گرداند و وظیفه برنامه‌نویس فقط جاگذاری مقادیر دریافتی خود در پارامتر های این تابع است
در این مثال از توابع get که توسط خود برنامه‌نویس نوشته میشود استفاده می‌کنیم که همانطور که گفته شد مقادیر دریافتی را به عنوان مقدار به خروجی تابع دادیم که در نتیجه این تابع داده‌ها را به رابط کاربری ارسال می‌کند
## چند نکته مهم:
۱-اگر با برخی مقادیر خروجی تابع کاری ندارید مقادیر پیش‌فرض را تغییر ندهید.

۲-هرگز اسم توابع را عوض نکنید
# شرکت کنندگان لیگ rov  باید توابع زیر را پیاده سازی کنند:
تابع fetch_raw_imu برای ژیروسکوپ و قطب‌نما

تابع fetch_gps_raw_int برای جی پی اس

تابع fetch_scaled_pressure برای سنسور فشار

# شرکت کنندگان لیگ usv  باید توابع زیر را پیاده سازی کنند:
تابع fetch_raw_imu برای ژیروسکوپ و قطب‌نما

تابع fetch_gps_raw_int برای جی پی اس

تابع fetch_scaled_pressure برای سنسور فشار

توضیحات بیشتر در مورد هر تابع در زیر آن آورده شده است




