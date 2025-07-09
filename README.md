# توضیحات استاندارد ارتباط با رابط کاربری
برای پیاده‌سازی استاندارد از توابع داخل فایل fetch_functions.py استفاده می‌کنیم
## یک نمونه از پیاده‌سازی:
```python
def fetch_raw_imu_data():
#تابع نوشته شده توسط برنامه‌نویس برای مثال تابع get_compass_data
  compass_data=get_compass_data()
  gyro_data=get_gyro_data()
  accel_data=get_accel_data()

    return {
        "xacc":accel_data["x"],
        "yacc":accel_data["y"],
        "zacc":accel_data["z"],
        "xgyro":gyro_data["x"],
        "ygyro":gyro_data["y"],
        "zgyro":gyro_data["z"],
        "xmag":compass_data["x"],
        "ymag":compass_data["y],
        "zmag":compass_data["z"],
        "id":0,
        "temperature":0,
    }
```
ساختار توابع fetch به این صورت است که یک دیکشنری را برمی‌گرداند و وظیفه برنامه‌نویس فقط جاگذاری مقادیر دریافتی خود در این دیکشنری است
در این مثال از توابع get که توسط خود برنامه‌نویس نوشته میشود استفاده می‌کنیم که همانطور که گفته شد مقادیر دریافتی را به عنوان مقدار به دیکشنری دادیم که در نتیجه این تابع داده‌ها را به رابط کاربری ارسال می‌کند
## چند نکته مهم:
۱-اگر با برخی مقادیر دیکشنری کاری ندارید مقادیر پیش‌فرض را تغییر ندهید.

۲-هرگز اسم توابع را عوض نکنید
# شرکت کنندگان لیگ rov  باید توابع زیر را پیاده سازی کنند:
تابع fetch_raw_imu برای ژیروسکوپ و قطب‌نما

تابع fetch_gps_raw_int برای جی پی اس

تابع fetch_scaled_pressure برای سنسور فشار

# شرکت کنندگان لیگ usv  باید توابع زیر را پیاده سازی کنند:
تابع fetch_raw_imu برای ژیروسکوپ و قطب‌نما

تابع fetch_gps_raw_int برای جی پی اس

تابع fetch_scaled_pressure برای سنسور فشار




