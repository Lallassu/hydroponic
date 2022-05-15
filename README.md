# Hydroponic
Hydroponic automation with ESP32 and sensors

- Temperature sensors  (DS18B20)
- 5V Relays

# Installation
Using platformio:

```
pio run -t upload 
```

Read the serial interface to get log output:
```
pio device monitor -b 115200
```

# Home Assistant Integration

Temperature sensors:
```
sensor:
    - platform: mqtt
      state_topic: 'hydroponic/temperature1'
      name: 'Temperature1'
    - platform: mqtt
      state_topic: 'hydroponic/temperature2'
      name: 'Temperature2'
```

Relay switches:
```
switch:
  - platform: rest
    name: "hydroponic_fan"
    method: "post"
    resource: http://192.168.1.205/fan
    body_on: '{"active": "true"}'
    body_off: '{"active": "false"}'
    is_on_template: "{{ value_json.is_active }}"
    headers:
      Content-Type: application/json
  - platform: rest
    name: "hydroponic_cam"
    method: "post"
    resource: http://192.168.1.205/cam
    body_on: '{"active": "true"}'
    body_off: '{"active": "false"}'
    is_on_template: "{{ value_json.is_active }}"
    headers:
      Content-Type: application/json
  - platform: rest
    name: "hydroponic_pump"
    method: "post"
    resource: http://192.168.1.205/pump
    body_on: '{"active": "true"}'
    body_off: '{"active": "false"}'
    is_on_template: "{{ value_json.is_active }}"
    headers:
      Content-Type: application/json
```

# License
MIT
