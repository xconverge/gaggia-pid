# gaggia-pid
ESP8266 PID controller for Gaggia Classic espresso machine

Query target temp, uptime, and actual temp:

```
curl -v xxx.xxx.xxx.xxx/json
```

Set the new target temp:

```
curl -d "setpoint=200" -X POST xxx.xxx.xxx.xxx/set
```

Set PID parameters:

```
curl -d "kp=1.0&ki=0.5&kd=0.1" -X POST xxx.xxx.xxx.xxx/set
```

Save config to EEPROM:

```
curl -X POST xxx.xxx.xxx.xxx/save
```