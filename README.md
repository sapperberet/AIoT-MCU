# Smart Home MQTT Messages

This document defines the MQTT messages used in the smart home system.

-   **Actuators:**\
    **المتحكم (ESP) يستقبل هذه الرسائل من الـ MQTT Broker لتنفيذ
    الأوامر.**

-   **Sensors:**\
    **المتحكم (ESP) يرسل هذه الرسائل إلى الـ MQTT Broker بعد قراءة القيم
    من الحساسات.**

------------------------------------------------------------------------

## 🔧 Actuators

### *(المتحكم يستقبل هذه الرسائل)*

------------------------------------------------------------------------

### 𖣘 Fan

**Topic:**
    
    home/actuators/fan

**Message Content:**

    in / out / on / off

------------------------------------------------------------------------

### 💡 Lights -- Floor 1

**Topic:**

    home/actuators/lights/floor1

**Message Content:**

    on / off

------------------------------------------------------------------------

### 💡 Lights -- Floor 2

**Topic:**

    home/actuators/lights/floor2

**Message Content:**

    on / off

------------------------------------------------------------------------

### 💡 Lights -- Landscape

**Topic:**
    
    home/actuators/lights/landscape

**Message Content:**

    on / off

------------------------------------------------------------------------

### 💡 Lights -- RGB

**Topic:**

    home/actuators/lights/rgb

**Message Content:**

    b <int_value_of_brightness> / c <string_value_of_color>

------------------------------------------------------------------------

### 🚨 Buzzer

**Topic:**

    home/actuators/buzzer

**Message Content:**

    on / off

------------------------------------------------------------------------

### 🚪 Garage Motor

**Topic:**

    home/actuators/motors/garage

**Message Content:**

    open / close

------------------------------------------------------------------------

### 🪟 Front Window Motor

**Topic:**

    home/actuators/motors/frontwindow

**Message Content:**

    open / close

------------------------------------------------------------------------

### 🪟 Side Window Motor

**Topic:**

    home/actuators/motors/sidewindow

**Message Content:**

    open / close

------------------------------------------------------------------------

### 🚪 Door Motor

 **Topic:**
 
    home/actuators/motors/door

**Message Content:**

    open / close

------------------------------------------------------------------------

## 🌡️ Sensors

### *(المتحكم يرسل هذه الرسائل)*

------------------------------------------------------------------------

### Gas Sensor

**Topic:**

    home/sensors/gas

**Message Content:**

    <gas_sensor_value>

------------------------------------------------------------------------

### LDR Sensor

**Topic:**

    home/sensors/ldr

**Message Content:**

    <ldr_sensor_value>

------------------------------------------------------------------------

### Rain Sensor

**Topic:**

    home/sensors/rain

**Message Content:**

    <rain_sensor_value>

------------------------------------------------------------------------

### Voltage Sensor

**Topic:**

    home/sensors/voltage

**Message Content:**

    <voltage_value>

------------------------------------------------------------------------

### Current Sensor

**Topic:**

    home/sensors/current

**Message Content:**

    <current_value>

------------------------------------------------------------------------

### Humidity Sensor

**Topic:**

    home/sensors/humidity

**Message Content:**

    <humidity_value>
