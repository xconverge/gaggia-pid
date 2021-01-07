
Query target temp, uptime, and actual temp:

```
curl -v xxx.xxx.xxx.xxx/json
```

Set the new target temp:

```
curl -d "setpoint=200" -X POST xxx.xxx.xxx.xxx/set
```